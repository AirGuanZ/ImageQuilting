#include <map>

#include <agz/utility/console.h>
#include <agz/utility/misc.h>

#include "imageQuilting.h"

namespace
{

    float computeErrorSum(
        const Image<Float3> &A, int xA, int yA,
        const Image<Float3> &B, int xB, int yB,
        int width, int height)
    {
        float sqrErrSum = 0;
        for(int ix = 0, ixA = xA, ixB = xB; ix < width;
            ++ix, ++ixA, ++ixB)
        {
            for(int iy = 0, iyA = yA, iyB = yB; iy < height;
                ++iy, ++iyA, ++iyB)
            {
                sqrErrSum += agz::math::sqr(
                    (A(iyA, ixA) - B(iyB, ixB)).lum());
            }
        }
        return sqrErrSum;
    }

    template<bool DST_X_POSITIVE, bool DST_Y_POSITIVE>
    float computeMSE(
        const Image<Float3> &src,
        const Image<Float3> &dst,
        int srcX, int srcY,
        int dstX, int dstY,
        int blockW,
        int blockH,
        int overlapW,
        int overlapH)
    {
        if constexpr(!DST_X_POSITIVE && !DST_Y_POSITIVE)
            return 0;

        if constexpr(DST_X_POSITIVE && !DST_Y_POSITIVE)
        {
            const float sqrErrSum = computeErrorSum(
                src, srcX, srcY, dst, dstX, dstY, overlapW, blockH);

            const int pixelCnt = overlapW * blockH;

            return sqrErrSum / pixelCnt;
        }

        if constexpr(!DST_Y_POSITIVE && DST_Y_POSITIVE)
        {
            const float sqrErrSum = computeErrorSum(
                src, srcX, srcY, dst, dstX, dstY, blockW, overlapH);

            const int pixelCnt = overlapH * blockW;

            return sqrErrSum / pixelCnt;
        }

        const float A = computeErrorSum(
            src, srcX, srcY, dst, dstX, dstY, overlapW, overlapH);

        const float B = computeErrorSum(
            src, srcX, srcY + overlapH, dst, dstX, dstY + overlapH,

            overlapW, blockH - overlapH);
        const float C = computeErrorSum(
            src, srcX + overlapW, srcY, dst, dstX + overlapW, dstY,
            blockW - overlapW, overlapH);

        const int pixelCnt = overlapW * overlapH
                           + overlapW * (blockH - overlapH)
                           + overlapH * (blockW - overlapW);

        return (A + B + C) / pixelCnt;
    }

    struct MinCostCutUnit
    {
        float cost         = 0;
        int lastUnitOffset = 0;
    };

    Image<MinCostCutUnit> computeVerticalCutCost(
        const Image<Float3> &A,
        const Image<Float3> &B,
        int xA,    int yA,
        int xB,    int yB,
        int width, int height)
    {
        // result(y, x) is min cost cut which passes
        //      Edge(A(xA + x, yA + y), B(xB + x + 1, yB + y))
        Image<MinCostCutUnit> result(height, width - 1);

        auto edgeCost = [&](int x, int y)
        {
            return agz::math::sqr(
                (A(yA + y, xA + x) - B(yB + y, xB + x + 1)).lum());
        };

        for(int xi = 0; xi < width - 1; ++xi)
            result(0, xi).cost = edgeCost(xi, 0);

        for(int yi = 0; yi < height; ++yi)
        {
            for(int xi = 0; xi < width - 1; ++xi)
            {
                const float costLeft = xi > 0 ?
                    edgeCost(xi - 1, yi) : std::numeric_limits<float>::max();
                const float costMiddle =
                    edgeCost(xi, yi);
                const float costRight = xi < width - 2 ?
                    edgeCost(xi + 1, yi) : std::numeric_limits<float>::max();

                if(costLeft <= costMiddle && costLeft <= costRight)
                    result(yi, xi) = { costLeft, -1 };
                else if(costMiddle <= costLeft && costMiddle <= costRight)
                    result(yi, xi) = { costMiddle, 0 };
                else
                    result(yi, xi) = { costRight, 1 };
            }
        }

        return result;
    }

    Image<MinCostCutUnit> computeHorizontalCutCost(
        const Image<Float3> &A,
        const Image<Float3> &B,
        int xA,    int yA,
        int xB,    int yB,
        int width, int height)
    {
        // result(y, x) is min cost cut which passes
        //      Edge(A(xA + x, yA + y), B(xB + x, yB + y + 1))
        Image<MinCostCutUnit> result(height - 1, width);

        auto edgeCost = [&](int x, int y)
        {
            return agz::math::sqr(
                (A(yA + y, xA + x) - B(yB + y + 1, xB + x)).lum());
        };

        for(int yi = 0; yi < height - 1; ++yi)
            result(yi, 0).cost = edgeCost(0, yi);

        for(int xi = 0; xi < width; ++xi)
        {
            for(int yi = 0; yi < height - 1; ++yi)
            {
                const float costNY = yi > 0 ?
                    edgeCost(xi, yi - 1) : std::numeric_limits<float>::max();
                const float costMY = edgeCost(xi, yi);
                const float costPY = yi < height - 2 ?
                    edgeCost(xi, yi + 1) : std::numeric_limits<float>::max();

                if(costNY <= costMY && costNY <= costPY)
                    result(yi, xi) = { costNY, -1 };
                else if(costMY <= costNY && costMY <= costPY)
                    result(yi, xi) = { costMY, 0 };
                else
                    result(yi, xi) = { costPY, 1 };
            }
        }

        return result;
    }

    /**
     * ret[yi]: the cut passes
     *      Edge(A(xA + ret[yi], yA + yi), B(xB + ret[yi] + 1, yB + yi))
     */
    std::vector<int> computeVerticalMinCostCut(
        const Image<Float3> &A,
        const Image<Float3> &B,
        int xA,    int yA,
        int xB,    int yB,
        int width, int height)
    {
        const auto costs = computeVerticalCutCost(
            A, B, xA, yA, xB, yB, width, height);

        float bestCost  = std::numeric_limits<float>::max();
        int bestCostXi = -1, nextXi = -1;
        for(int xi = 0; xi < width - 1; ++xi)
        {
            if(costs(height - 1, xi).cost < bestCost)
            {
                bestCost   = costs(height - 1, xi).cost;
                bestCostXi = xi;
                nextXi     = xi + costs(height - 1, xi).lastUnitOffset;
            }
        }

        std::vector<int> ret(height);
        ret[height - 1] = bestCostXi;

        for(int yi = height - 2; yi >= 0; --yi)
        {
            ret[yi] = nextXi;
            nextXi = nextXi + costs(yi, nextXi).lastUnitOffset;
        }

        return ret;
    }

    /**
     * ret[xi]: the cut passes
     *      Edge(A(xA + xi, yA + ret[xi]), B(xB + xi, yB + ret[xi] + 1))
     */
    std::vector<int> computeHorizontalMinCostCut(
        const Image<Float3> &A,
        const Image<Float3> &B,
        int xA,    int yA,
        int xB,    int yB,
        int width, int height)
    {
        const auto costs = computeHorizontalCutCost(
            A, B, xA, yA, xB, yB, width, height);

        float bestCost = std::numeric_limits<float>::max();
        int bestCostYi = -1, nextYi = -1;
        for(int yi = 0; yi < height - 1; ++yi)
        {
            if(costs(yi, width - 1).cost < bestCost)
            {
                bestCost   = costs(yi, width - 1).cost;
                bestCostYi = yi;
                nextYi     = yi + costs(yi, width - 1).lastUnitOffset;
            }
        }

        std::vector<int> ret(width);
        ret[width - 1] = bestCostYi;

        for(int xi = width - 2; xi >= 0; --xi)
        {
            ret[xi] = nextYi;
            nextYi = nextYi + costs(nextYi, xi).lastUnitOffset;
        }

        return ret;
    }

} // namespace anonymous

ImageQuilting::ImageQuilting()
    : blockW_  (3), blockH_(3),
      overlapW_(1), overlapH_(1),
      blockTolerance_(0.1f),
      useMSEBlockSelection_(true),
      useMinEdgeCut_(true)
{
    
}

void ImageQuilting::setParams(int blockWidth, int blockHeight) noexcept
{
    setParams(
        blockWidth,
        blockHeight,
        std::max(1, blockWidth / 6),
        std::max(1, blockHeight / 6),
        0.1f);
}

void ImageQuilting::setParams(
    int   blockWidth,
    int   blockHeight,
    int   overlapWidth,
    int   overlapHeight,
    float blockTolerance) noexcept
{
    blockW_         = blockWidth;
    blockH_         = blockHeight;
    overlapW_       = overlapWidth;
    overlapH_       = overlapHeight;
    blockTolerance_ = blockTolerance;
}

void ImageQuilting::UseMSEBlockSelection(bool use) noexcept
{
    useMSEBlockSelection_ = use;
}

void ImageQuilting::UseMinEdgeCut(bool use) noexcept
{
    useMinEdgeCut_ = use;
}

Image<Float3> ImageQuilting::generate(
    const Image<Float3> &src,
    int                  generatedWidth,
    int                  generatedHeight) const
{
    // compute the padded image size
    // X * Y block
    //     -> (X * blockW - (X - 1) * overlapX)
    //      * (Y * blockH - (Y - 1) * overlapY) pixels
    //
    // (X * overlapY - (X - 1) * overlapX) >= generatedWidth
    //     -> X >= (generatedWidth - overlapX) / (overlapY - overlapX)
    //
    // (Y * blockH - (Y - 1) * overlapY) >= generatedHeight
    //     -> Y >= (generatedHeight - overlapY) / (blockH - overlapY)

    const int blockCountX = static_cast<int>(std::ceil(
        static_cast<float>(generatedWidth - overlapW_)
                        / (blockW_ - overlapW_)));
    const int blockCountY = static_cast<int>(std::ceil(
        static_cast<float>(generatedHeight - overlapH_)
                        / (blockH_ - overlapH_)));

    const int imageW = blockCountX * blockW_ - (blockCountX - 1) * overlapW_;
    const int imageH = blockCountY * blockH_ - (blockCountY - 1) * overlapH_;

    // create dst image

    Image<Float3> dst(imageH, imageW);

    // fill blocks of dst image with simple raster scan order

    std::default_random_engine rng{ std::random_device()() };

    agz::console::progress_bar_t pbar(blockCountY * blockCountX, 80, '=');
    pbar.display();

    for(int blockY = 0; blockY < blockCountY; ++blockY)
    {
        const int y = blockY * (blockH_ - overlapH_);

        for(int blockX = 0; blockX < blockCountX; ++blockX)
        {
            const int x = blockX * (blockW_ - overlapW_);

            const auto block = pickSourceBlock(src, dst, x, y, rng);
            putBlockAt(block, dst, x, y);

            ++pbar;
        }

        pbar.display();
    }

    pbar.done();

    return dst.subtex(0, generatedHeight, 0, generatedWidth);
}

Image<Float3> ImageQuilting::pickSourceBlock(
    const Image<Float3>         &src,
    const Image<Float3>         &dst,
    int                          x,
    int                          y,
    std::default_random_engine  &rng) const
{
    if(!useMSEBlockSelection_)
    {
        const std::uniform_int_distribution disX(
            0, static_cast<int>(src.width() - blockW_ - 1));
        const std::uniform_int_distribution disY(
            0, static_cast<int>(src.height() - blockH_ - 1));

        const int srcX = disX(rng);
        const int srcY = disY(rng);

        return src.subtex(srcY, srcY + blockH_, srcX, srcX + blockW_);
    }

    std::multimap<float, agz::math::vec2i> mseToXY;

    AGZ_DYN_FUNC_BOOL2_TEMPLATE(computeMSE, computeMSEFunc);
    const auto mseFunc = computeMSEFunc[
        agz::misc::bools_to_dyn_call_idx({ x > 0, y > 0 })];

    for(int srcY = 0; srcY < src.height() - blockH_; ++srcY)
    {
        for(int srcX = 0; srcX < src.width() - blockW_; ++srcX)
        {
            const float mse = mseFunc(
                src, dst, srcX, srcY, x, y,
                blockW_, blockH_, overlapW_, overlapH_);

            if((x == 0 && y == 0) || mse > 0.001f)
                mseToXY.insert({ mse, agz::math::vec2i{ srcX, srcY } });
        }
    }

    const float maxAllowedMSE = mseToXY.begin()->first * (1 + blockTolerance_);
    std::vector<agz::math::vec2i> allowedXYs;

    for(auto &p : mseToXY)
    {
        if(p.first <= maxAllowedMSE)
            allowedXYs.push_back(p.second);
        else
            break;
    }

    const std::uniform_int_distribution dis(
        0, static_cast<int>(allowedXYs.size() - 1));
    const auto xy = allowedXYs[dis(rng)];

    return src.subtex(xy.y, xy.y + blockH_, xy.x, xy.x + blockW_);
}

void ImageQuilting::putBlockAt(
    const Image<Float3> &block,
    Image<Float3>       &dst,
    int                  x,
    int                  y) const
{
    if(!useMinEdgeCut_)
    {
        for(int yi = 0; yi < block.height(); ++yi)
        {
            for(int xi = 0; xi < block.width(); ++xi)
            {
                dst(y + yi, x + xi) = block(yi, xi);
            }
        }
        return;
    }

    if(x > 0 && y == 0)
    {
        const auto verticalCut = computeVerticalMinCostCut(
            dst, block, x, y, 0, 0, overlapW_, blockH_);

        for(int yi = 0; yi < block.height(); ++yi)
        {
            for(int xi = 0; xi < block.width(); ++xi)
            {
                if(xi > verticalCut[yi])
                    dst(y + yi, x + xi) = block(yi, xi);
            }
        }
    }
    else if(x == 0 && y > 0)
    {
        const auto horizontalCut = computeHorizontalMinCostCut(
            dst, block, x, y, 0, 0, blockW_, overlapH_);

        for(int yi = 0; yi < block.height(); ++yi)
        {
            for(int xi = 0; xi < block.width(); ++xi)
            {
                if(yi > horizontalCut[xi])
                    dst(y + yi, x + xi) = block(yi, xi);
            }
        }
    }
    else if(x > 0 && y > 0)
    {
        const auto verticalCut = computeVerticalMinCostCut(
            dst, block, x, y, 0, 0, overlapW_, blockH_);

        const auto horizontalCut = computeHorizontalMinCostCut(
            dst, block, x, y, 0, 0, blockW_, overlapH_);

        for(int yi = 0; yi < block.height(); ++yi)
        {
            for(int xi = 0; xi < block.width(); ++xi)
            {
                if(xi > verticalCut[yi] && yi > horizontalCut[xi])
                    dst(y + yi, x + xi) = block(yi, xi);
            }
        }
    }
    else
    {
        for(int yi = 0; yi < block.height(); ++yi)
        {
            for(int xi = 0; xi < block.width(); ++xi)
                dst(y + yi, x + xi) = block(yi, xi);
        }
    }
}

#pragma once

#include <random>

#include <agz-utils/texture.h>

using Float3 = agz::math::float3;

template<typename T>
using Image = agz::texture::texture2d_t<T>;

template<typename T>
using ImageView = agz::texture::texture2d_view_t<T, true>;

class ImageQuilting
{
public:

    ImageQuilting();

    void setParams(int blockWidth, int blockHeight) noexcept;

    void setParams(
        int   blockWidth,
        int   blockHeight,
        int   overlapWidth,
        int   overlapHeight,
        float blockTolerance) noexcept;

    void UseMSEBlockSelection(bool use) noexcept;

    void UseMinEdgeCut(bool use) noexcept;

    Image<Float3> generate(
        const Image<Float3> &src,
        int                  generatedWidth,
        int                  generatedHeight) const;

private:

    /**
     * randomly pick a block from src within given overlapped area MSE tolerance.
     * the result will be put at (x, y) in dst. all pixels before the new block
     * in scan order are assumed to be already filled.
     */
    ImageView<Float3> pickSourceBlock(
        const Image<Float3>        &src,
        const Image<Float3>        &dst,
        int                         x,
        int                         y,
        std::default_random_engine &rng) const;

    /**
     * put block at (x, y) in dst with minimum err boundary cut
     */
    void putBlockAt(
        const ImageView<Float3> &block,
        Image<Float3>           &dst,
        int                      x,
        int                      y) const;

    int blockW_;
    int blockH_;
    int overlapW_;
    int overlapH_;

    float blockTolerance_;

    bool useMSEBlockSelection_;
    bool useMinEdgeCut_;
};

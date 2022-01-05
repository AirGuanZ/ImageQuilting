#include <agz-utils/file.h>
#include <agz-utils/image.h>
#include <agz-utils/string.h>

#include <cxxopts.hpp>

#include "imageQuilting.h"

struct CmdArgs
{
    std::string inputFilename;
    std::string outputFilename;

    int outputWidth  = 0;
    int outputHeight = 0;

    int blockWidth  = 0;
    int blockHeight = 0;

    int overlapWidth  = 0;
    int overlapHeight = 0;

    bool useMSEBlockSelection = false;
    bool useMinEdgeCut        = false;

    float blockTolerance = 0;
};

std::optional<CmdArgs> parseCmdArgs(int argc, char *argv[])
{
    cxxopts::Options options("ImageQuilting");
    options.add_options()
        ("input",      "input image filename",      cxxopts::value<std::string>())
        ("output",     "output image filename",     cxxopts::value<std::string>())
        ("width",      "output image width",        cxxopts::value<int>())
        ("height",     "output image height",       cxxopts::value<int>())
        ("blockW",     "block width",               cxxopts::value<int>())
        ("blockH",     "block height",              cxxopts::value<int>())
        ("overlapW",   "overlapped area width",     cxxopts::value<int>())
        ("overlapH",   "overlapped area height",    cxxopts::value<int>())
        ("mseBlock",   "use MSE block selection",   cxxopts::value<bool>())
        ("minCutEdge", "use min cost cut edge",     cxxopts::value<bool>())
        ("tolerance",  "block selection tolerance", cxxopts::value<float>())
        ("help",       "help information");

    const auto args = options.parse(argc, argv);

    if(args.count("help"))
    {
        std::cout << options.help({ "" }) << std::endl;
        return {};
    }

    CmdArgs result;

    try
    {
        result.inputFilename  = args["input"] .as<std::string>();
        result.outputFilename = args["output"].as<std::string>();
        result.outputWidth    = args["width"] .as<int>();
        result.outputHeight   = args["height"].as<int>();
        result.blockWidth     = args["blockW"].as<int>();
        result.blockHeight    = args["blockH"].as<int>();

        if(args.count("overlapW"))
            result.overlapWidth = args["overlapW"].as<int>();
        else
            result.overlapWidth = std::max(1, result.blockWidth / 6);

        if(args.count("overlapH"))
            result.overlapHeight = args["overlapH"].as<int>();
        else
            result.overlapHeight = std::max(1, result.blockHeight / 6);

        if(args.count("mseBlock"))
            result.useMSEBlockSelection = args["mseBlock"].as<bool>();
        else
            result.useMSEBlockSelection = true;

        if(args.count("minCutEdge"))
            result.useMinEdgeCut = args["minCutEdge"].as<bool>();
        else
            result.useMinEdgeCut = true;

        if(args.count("tolerance"))
            result.blockTolerance = args["tolerance"].as<float>();
        else
            result.blockTolerance = 0.1f;
    }
    catch(...)
    {
        std::cout << options.help({ "" }) << std::endl;
        return {};
    }

    return result;
}

void run(int argc, char *argv[])
{
    using namespace agz;
    using namespace img;
    using namespace stdstr;

    const auto args = parseCmdArgs(argc, argv);
    if(!args)
        return;

    ImageQuilting imageQuilting;
    imageQuilting.setParams(
        args->blockWidth,   args->blockHeight,
        args->overlapWidth, args->overlapHeight,
        args->blockTolerance);
    imageQuilting.UseMSEBlockSelection(args->useMSEBlockSelection);
    imageQuilting.UseMinEdgeCut(args->useMinEdgeCut);
    
    auto src = Image<Float3>(load_rgb_from_file(
        args->inputFilename).map(
            [](const math::color3b &c)
    {
        return Float3(math::from_color3b<float>(c));
    }));

    auto dst = imageQuilting.generate(
        src, args->outputWidth, args->outputHeight);

    auto dstu8 = dst.map([](const Float3 &c)
    {
        return math::to_color3b<float>(c);
    }).get_data();

    file::create_directory_for_file(args->outputFilename);

    const std::string lowerOutput = to_lower(args->outputFilename);
    if(ends_with(lowerOutput, ".png"))
        save_rgb_to_png_file(args->outputFilename, dstu8);
    else if(ends_with(lowerOutput, ".jpg"))
        save_rgb_to_jpg_file(args->outputFilename, dstu8);
    else if(ends_with(lowerOutput, ".bmp"))
        save_rgb_to_bmp_file(args->outputFilename, dstu8);
    else
        throw std::runtime_error(
            "unsupported output format: " + args->outputFilename);

    std::cout << "Generation complete..." << std::endl;
}

int main(int argc, char *argv[])
{
    try
    {
        run(argc, argv);
    }
    catch(const std::exception &err)
    {
        std::cerr << err.what() << std::endl;
        return -1;
    }
}

#include <QCoreApplication>
#include <QCommandLineParser>

#include <iostream>
#include <chrono>

#include "mesh.h"


int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);
    QCommandLineParser parser;
    parser.addHelpOption();
    parser.addPositionalArgument("infile",  "Input .obj file path");
    parser.addPositionalArgument("outfile", "Output .obj file path");
    parser.addPositionalArgument("method",  "subdivide/simplify/remesh/denoise");

    // Subdivide: number of iterations
    // Simplify:  number of faces to remove
    // Remesh:    number of iterations
    // Denoise:   number of iterations
    parser.addPositionalArgument("args1", "respective argument for the method");

    // Remesh: Tangential smoothing weight
    // Denoise: Smoothing parameter 1 (\Sigma_c)
    parser.addPositionalArgument("args2", "respective argument2 for the method");

    // Denoise: Smoothing parameter 2 (\Sigma_s)
    parser.addPositionalArgument("args3", "respective argument3 for the method");

    // Denoise: Kernel size (\rho)
    parser.addPositionalArgument("args4", "respective argument4 for the method");

    parser.process(a);

    // Check for invalid argument count
    const QStringList args = parser.positionalArguments();
    if (args.size() < 4) {
        std::cerr << "Arguments <input .obj file path> <output .obj file path> <method (subdivide, simplify, or denoise)> <method-specific arguments ...>" << std::endl;
        a.exit(1);
        return 1;
    }

    // Parse common inputs
    QString infile  = args[0];
    QString outfile = args[1];
    QString method  = args[2];

    // Load
    Mesh m;
    m.loadFromFile(infile.toStdString());

    // Start timing
    auto t0 = std::chrono::high_resolution_clock::now();

    // Switch on method
    if (method == "subdivide") {

        // TODO

    } else if (method == "simplify") {

        // TODO

    } else if (method == "noise") {

        // TODO

    } else if (method == "denoise") {

        // TODO

    } else {

        std::cerr << "Error: Unknown method \"" << method.toUtf8().constData() << "\"" << std::endl;

    }

    // Finish timing
    auto t1 = std::chrono::high_resolution_clock::now();
    auto duration = duration_cast<std::chrono::milliseconds>(t1 - t0).count();
    std::cout << "Execution took " << duration << " milliseconds." << std::endl;

    // Save
    m.saveToFile(outfile.toStdString());

    a.exit();
}

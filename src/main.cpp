#include "App.h"

#include <cstdlib>

int main(int argc, char** argv)
{
    App app;
    if (!app.init(argc > 1 ? argv[1] : nullptr))
        return EXIT_FAILURE;
    app.run();
    app.shutdown();
    return EXIT_SUCCESS;
}

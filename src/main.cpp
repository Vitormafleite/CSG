#include "App.h"

#include <cstdlib>

int main()
{
    App app;
    if (!app.init())
        return EXIT_FAILURE;
    app.run();
    app.shutdown();
    return EXIT_SUCCESS;
}

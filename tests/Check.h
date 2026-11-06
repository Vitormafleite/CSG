#pragma once

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

// Minimal test helpers shared by the test files.
inline int g_failures = 0;

#define CHECK(cond)                                                          \
    do                                                                       \
    {                                                                        \
        if (!(cond))                                                         \
        {                                                                    \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);      \
            ++g_failures;                                                    \
        }                                                                    \
    } while (0)

inline bool near(double a, double b, double relTol)
{
    return std::fabs(a - b) <= relTol * std::max(std::fabs(b), 1e-9);
}

inline void checkNear(const char* what, double got, double want, double relTol)
{
    bool ok = near(got, want, relTol);
    std::printf("%s %-44s got %.5f want %.5f\n", ok ? "ok  " : "FAIL", what, got, want);
    if (!ok)
        ++g_failures;
}

// Counts failures of a property over many random trials and reports once.
struct Property
{
    std::string name;
    int trials = 0, failures = 0;
    double worst = 0.0;

    explicit Property(std::string n) : name(std::move(n)) {}
    void record(bool ok, double error = 0.0)
    {
        ++trials;
        if (!ok)
            ++failures;
        worst = std::max(worst, error);
    }
    ~Property()
    {
        std::printf("%s %-44s %d/%d failed, worst error %.2e\n", failures ? "FAIL" : "ok  ", name.c_str(), failures, trials,
                    worst);
        if (failures)
            ++g_failures;
    }
};

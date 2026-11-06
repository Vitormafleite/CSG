#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <cmath>
#include <vector>

namespace csg {

inline glm::vec3 interpolate(const glm::vec3& a, const glm::vec3& b, float f) { return glm::mix(a, b, f); }
inline glm::quat interpolate(const glm::quat& a, const glm::quat& b, float f) { return glm::slerp(a, b, f); }

// Keyframed value over time (the 4th dimension). Always holds at least one key;
// with a single key the value is constant.
template <class T>
class Track
{
public:
    struct Key
    {
        float time;
        T value;
    };

    static constexpr float kTimeEps = 1e-4f;

    explicit Track(const T& value = T()) : m_keys{ { 0.0f, value } } {}

    const std::vector<Key>& keys() const { return m_keys; }
    bool animated() const { return m_keys.size() > 1; }

    T value(float t) const
    {
        if (t <= m_keys.front().time)
            return m_keys.front().value;
        if (t >= m_keys.back().time)
            return m_keys.back().value;
        auto next = std::upper_bound(m_keys.begin(), m_keys.end(), t,
                                     [](float time, const Key& k) { return time < k.time; });
        auto prev = next - 1;
        float f = (t - prev->time) / (next->time - prev->time);
        return interpolate(prev->value, next->value, f);
    }

    // Writes a value at time t. A constant track is simply overwritten unless
    // keying is requested; an animated track always gets a key at t.
    void set(float t, const T& value, bool key)
    {
        if (!key && !animated())
        {
            m_keys.front().value = value;
            return;
        }
        auto it = std::lower_bound(m_keys.begin(), m_keys.end(), t,
                                   [](const Key& k, float time) { return k.time < time - kTimeEps; });
        if (it != m_keys.end() && std::fabs(it->time - t) <= kTimeEps)
            it->value = value;
        else
            m_keys.insert(it, Key{ t, value });
    }

    bool hasKeyAt(float t) const
    {
        for (const Key& k : m_keys)
            if (std::fabs(k.time - t) <= kTimeEps)
                return true;
        return false;
    }

    void removeKeyAt(float t)
    {
        if (m_keys.size() < 2)
            return;
        m_keys.erase(std::remove_if(m_keys.begin(), m_keys.end(),
                                    [t](const Key& k) { return std::fabs(k.time - t) <= kTimeEps; }),
                     m_keys.end());
    }

    // Replaces all keys (sorted by time; keys at the same time keep the last). Ignores an empty list.
    void setKeys(std::vector<Key> keys)
    {
        if (keys.empty())
            return;
        std::stable_sort(keys.begin(), keys.end(), [](const Key& a, const Key& b) { return a.time < b.time; });
        m_keys.clear();
        for (const Key& k : keys)
        {
            if (!m_keys.empty() && std::fabs(m_keys.back().time - k.time) <= kTimeEps)
                m_keys.back() = k;
            else
                m_keys.push_back(k);
        }
    }

    // Collapses the track to the value at time t.
    void clearKeys(float t) { m_keys = { { 0.0f, value(t) } }; }

private:
    std::vector<Key> m_keys;
};

} // namespace csg

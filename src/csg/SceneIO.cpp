#include "csg/SceneIO.h"

#include "csg/Mesh.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace fs = std::filesystem;

namespace csg {

namespace {

const char* kPrimWords[] = { "block", "sphere", "cylinder", "cone", "mesh" };

// Shortest decimal that reads back as the same float.
std::string num(float v)
{
    char buf[32];
    for (int digits = 6; digits <= 9; ++digits)
    {
        std::snprintf(buf, sizeof(buf), "%.*g", digits, double(v));
        if (std::strtof(buf, nullptr) == v)
            break;
    }
    return buf;
}

std::string quoted(const std::string& s)
{
    std::string out = "\"";
    for (char c : s)
    {
        if (c == '"' || c == '\\')
            out += '\\';
        out += c;
    }
    return out + "\"";
}

// Path written into the file: relative to the .txt folder when possible.
std::string relativePath(const std::string& path, const std::string& baseDir)
{
    std::error_code ec;
    fs::path abs = fs::absolute(path, ec);
    fs::path base = fs::absolute(baseDir, ec);
    fs::path rel = abs.lexically_normal().lexically_relative(base.lexically_normal());
    return rel.empty() ? path : rel.generic_string();
}

void writeNode(std::ostringstream& out, const Node& n, int depth, const std::string& baseDir)
{
    std::string pad(size_t(depth) * 2, ' ');
    if (!n.isPrimitive())
    {
        const char* op = n.type == NodeType::Union ? "union" : n.type == NodeType::Intersection ? "intersection" : "difference";
        out << pad << op << " " << quoted(n.name) << "\n";
        writeNode(out, *n.a, depth + 1, baseDir);
        writeNode(out, *n.b, depth + 1, baseDir);
        out << pad << "end\n";
        return;
    }

    out << pad << "primitive " << kPrimWords[int(n.prim)] << " " << quoted(n.name) << "\n";
    std::string in = pad + "  ";
    if (n.prim == PrimType::Mesh)
        out << in << "file " << quoted(relativePath(n.meshPath, baseDir)) << "\n";
    out << in << "color " << num(n.color.r) << " " << num(n.color.g) << " " << num(n.color.b) << "\n";
    out << in << "reflectivity " << num(n.reflectivity) << "\n";
    if (n.limitedLife)
        out << in << "life " << num(n.visibleFrom) << " " << num(n.visibleTo) << "\n";
    for (const auto& k : n.position.keys())
        out << in << "position " << num(k.time) << "  " << num(k.value.x) << " " << num(k.value.y) << " " << num(k.value.z) << "\n";
    for (const auto& k : n.rotation.keys())
        out << in << "rotation " << num(k.time) << "  " << num(k.value.w) << " " << num(k.value.x) << " " << num(k.value.y) << " "
            << num(k.value.z) << "\n";
    for (const auto& k : n.size.keys())
        out << in << "size " << num(k.time) << "  " << num(k.value.x) << " " << num(k.value.y) << " " << num(k.value.z) << "\n";
    out << pad << "end\n";
}

// ---------------------------------------------------------------- parsing

struct Line
{
    int number;
    std::vector<std::string> words;
    std::vector<bool> quoted;
};

bool tokenize(const std::string& text, std::vector<Line>& lines, std::string& error)
{
    std::istringstream in(text);
    std::string raw;
    int number = 0;
    while (std::getline(in, raw))
    {
        ++number;
        Line line{ number, {}, {} };
        size_t i = 0;
        while (i < raw.size())
        {
            char c = raw[i];
            if (c == '#')
                break;
            if (std::isspace(static_cast<unsigned char>(c)))
            {
                ++i;
                continue;
            }
            std::string word;
            if (c == '"')
            {
                bool closed = false;
                for (++i; i < raw.size(); ++i)
                {
                    if (raw[i] == '\\' && i + 1 < raw.size())
                        word += raw[++i];
                    else if (raw[i] == '"')
                    {
                        closed = true;
                        ++i;
                        break;
                    }
                    else
                        word += raw[i];
                }
                if (!closed)
                {
                    error = "line " + std::to_string(number) + ": unterminated string";
                    return false;
                }
                line.words.push_back(word);
                line.quoted.push_back(true);
                continue;
            }
            while (i < raw.size() && !std::isspace(static_cast<unsigned char>(raw[i])) && raw[i] != '#')
                word += raw[i++];
            line.words.push_back(word);
            line.quoted.push_back(false);
        }
        if (!line.words.empty())
            lines.push_back(std::move(line));
    }
    return true;
}

class Parser
{
public:
    Parser(const std::vector<Line>& lines, const std::string& baseDir) : m_lines(lines), m_baseDir(baseDir) {}

    bool parse(std::vector<std::unique_ptr<Node>>& roots, float& duration)
    {
        m_duration = duration;
        if (!expectWord("CSG") || !numbers(1))
            return false;
        if (int(m_values[0]) != 1)
            return fail("unsupported version " + num(m_values[0]));
        next();
        while (!done())
        {
            const std::string& w = word(0);
            if (w == "duration")
            {
                if (!numbers(1) || m_values[0] <= 0.0f)
                    return fail("duration needs a positive number");
                duration = m_duration = m_values[0];
                next();
            }
            else if (w == "tree")
            {
                next();
                std::unique_ptr<Node> node;
                if (!parseNode(node))
                    return false;
                if (!expectWord("end"))
                    return false;
                next();
                roots.push_back(std::move(node));
            }
            else
                return fail("expected 'duration' or 'tree', got '" + w + "'");
        }
        return true;
    }

    const std::string& error() const { return m_error; }

private:
    const std::vector<Line>& m_lines;
    std::string m_baseDir;
    size_t m_at = 0;
    float m_duration = 5.0f;
    std::vector<float> m_values;
    std::string m_error;

    bool done() const { return m_at >= m_lines.size(); }
    void next() { ++m_at; }
    const std::string& word(size_t i) const { return m_lines[m_at].words[i]; }
    size_t count() const { return m_lines[m_at].words.size(); }

    bool fail(const std::string& message)
    {
        m_error = done() ? "end of file: " + message : "line " + std::to_string(m_lines[m_at].number) + ": " + message;
        return false;
    }

    bool expectWord(const char* w)
    {
        if (done())
            return fail(std::string("expected '") + w + "'");
        if (word(0) != w)
            return fail(std::string("expected '") + w + "', got '" + word(0) + "'");
        return true;
    }

    // Reads exactly n numbers after the keyword into m_values.
    bool numbers(size_t n)
    {
        m_values.clear();
        if (count() != n + 1)
            return fail("'" + word(0) + "' needs " + std::to_string(n) + " number(s)");
        for (size_t i = 1; i <= n; ++i)
        {
            try
            {
                size_t used = 0;
                m_values.push_back(std::stof(word(i), &used));
                if (used != word(i).size())
                    throw std::invalid_argument("");
            }
            catch (const std::exception&)
            {
                return fail("not a number: '" + word(i) + "'");
            }
        }
        return true;
    }

    bool name(std::string& out)
    {
        if (count() != 2 && count() != 3)
            return fail("expected a name in quotes");
        out = word(count() - 1);
        return true;
    }

    bool parseNode(std::unique_ptr<Node>& out)
    {
        if (done())
            return fail("expected a node");
        const std::string& w = word(0);
        auto node = std::make_unique<Node>();
        if (w == "union" || w == "intersection" || w == "difference")
        {
            node->type = w == "union" ? NodeType::Union : w == "intersection" ? NodeType::Intersection : NodeType::Difference;
            if (count() != 2 || !name(node->name))
                return fail("expected: " + w + " \"name\"");
            next();
            if (!parseNode(node->a) || !parseNode(node->b))
                return false;
            if (!expectWord("end"))
                return fail("an operation has exactly two subtrees, then 'end'");
            next();
            out = std::move(node);
            return true;
        }
        if (w != "primitive")
            return fail("expected 'primitive', 'union', 'intersection' or 'difference', got '" + w + "'");
        if (count() != 3)
            return fail("expected: primitive <type> \"name\"");
        int type = -1;
        for (int i = 0; i < 5; ++i)
            if (word(1) == kPrimWords[i])
                type = i;
        if (type < 0)
            return fail("unknown primitive type '" + word(1) + "'");
        node->type = NodeType::Primitive;
        node->prim = PrimType(type);
        node->name = word(2);
        next();

        std::vector<Track<glm::vec3>::Key> position, size;
        std::vector<Track<glm::quat>::Key> rotation;
        while (!done() && word(0) != "end")
        {
            const std::string& key = word(0);
            if (key == "color" && numbers(3))
                node->color = glm::clamp(glm::vec3(m_values[0], m_values[1], m_values[2]), 0.0f, 1.0f);
            else if (key == "reflectivity" && numbers(1))
                node->reflectivity = glm::clamp(m_values[0], 0.0f, 1.0f);
            else if (key == "life" && numbers(2))
            {
                node->limitedLife = true;
                node->visibleFrom = m_values[0];
                node->visibleTo = m_values[1];
            }
            else if (key == "position" && numbers(4))
                position.push_back({ m_values[0], glm::vec3(m_values[1], m_values[2], m_values[3]) });
            else if (key == "rotation" && numbers(5))
            {
                glm::quat q(m_values[1], m_values[2], m_values[3], m_values[4]);
                if (glm::length(q) < 1e-6f)
                    return fail("zero rotation quaternion");
                // Keep saved unit quaternions bit-exact; renormalize hand-written ones.
                if (std::fabs(glm::length(q) - 1.0f) > 1e-5f)
                    q = glm::normalize(q);
                rotation.push_back({ m_values[0], q });
            }
            else if (key == "size" && numbers(4))
                size.push_back({ m_values[0], glm::max(glm::vec3(m_values[1], m_values[2], m_values[3]), glm::vec3(0.0f)) });
            else if (key == "file" && node->prim == PrimType::Mesh)
            {
                if (count() != 2)
                    return fail("expected: file \"path\"");
                if (!loadMesh(*node, word(1)))
                    return false;
            }
            else
            {
                if (m_error.empty())
                    return fail("unknown property '" + key + "' for a " + kPrimWords[type]);
                return false;
            }
            next();
        }
        if (!expectWord("end"))
            return fail("primitive '" + node->name + "' is missing 'end'");
        if (node->prim == PrimType::Mesh && !node->mesh)
            return fail("mesh '" + node->name + "' has no 'file'");
        if (node->limitedLife && node->visibleTo < node->visibleFrom)
            return fail("life: 'to' is before 'from'");
        if (!node->limitedLife)
            node->visibleTo = m_duration; // the default range if lifetime is enabled later
        next();

        node->position.setKeys(std::move(position));
        node->rotation.setKeys(std::move(rotation));
        node->size.setKeys(std::move(size));
        out = std::move(node);
        return true;
    }

    bool loadMesh(Node& node, const std::string& file)
    {
        // Relative paths are relative to the .txt file, then to the working directory.
        fs::path p(file);
        std::string resolved = file;
        std::error_code ec;
        if (p.is_relative())
        {
            fs::path candidate = (fs::path(m_baseDir) / p).lexically_normal();
            if (fs::exists(candidate, ec))
                resolved = candidate.string();
        }
        std::string error;
        node.mesh = Mesh::load(resolved, error);
        if (!node.mesh)
            return fail(error);
        node.meshPath = resolved;
        return true;
    }
};

std::string directoryOf(const std::string& path)
{
    fs::path dir = fs::path(path).parent_path();
    return dir.empty() ? "." : dir.string();
}

} // namespace

std::string saveSceneText(const Scene& scene, const std::string& baseDir)
{
    std::ostringstream out;
    out << "# CSG modeler scene: one 'tree' block per root. Format: src/csg/SceneIO.h\n";
    out << "CSG 1\n";
    out << "duration " << num(scene.duration) << "\n";
    for (const auto& root : scene.roots)
    {
        out << "tree\n";
        writeNode(out, *root, 1, baseDir);
        out << "end\n";
    }
    return out.str();
}

bool saveScene(const Scene& scene, const std::string& path, std::string& error)
{
    std::ofstream file(path);
    if (!file)
    {
        error = "cannot write " + path;
        return false;
    }
    file << saveSceneText(scene, directoryOf(path));
    if (!file)
    {
        error = "write failed: " + path;
        return false;
    }
    return true;
}

bool loadSceneText(Scene& scene, const std::string& text, const std::string& baseDir, std::string& error)
{
    std::vector<Line> lines;
    if (!tokenize(text, lines, error))
        return false;
    std::vector<std::unique_ptr<Node>> roots;
    float duration = scene.duration;
    Parser parser(lines, baseDir);
    if (!parser.parse(roots, duration))
    {
        error = parser.error();
        return false;
    }
    scene.adopt(std::move(roots));
    scene.duration = duration;
    scene.time = std::min(scene.time, duration);
    return true;
}

bool loadScene(Scene& scene, const std::string& path, std::string& error)
{
    std::ifstream file(path);
    if (!file)
    {
        error = "cannot open " + path;
        return false;
    }
    std::stringstream buffer;
    buffer << file.rdbuf();
    return loadSceneText(scene, buffer.str(), directoryOf(path), error);
}

} // namespace csg

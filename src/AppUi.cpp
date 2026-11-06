#include "App.h"

#include "render/Gl.h"

#include "csg/Mesh.h"
#include "csg/SceneIO.h"

#include <GLFW/glfw3.h>
#include <imgui.h>

#include <glm/gtc/constants.hpp>

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <set>
#include <vector>

namespace {

constexpr ImGuiWindowFlags kPanelFlags = ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize |
                                         ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoSavedSettings |
                                         ImGuiWindowFlags_NoBringToFrontOnFocus;

const ImVec4 kInColor(0.25f, 0.85f, 0.35f, 1.0f);
const ImVec4 kOnColor(1.0f, 0.85f, 0.2f, 1.0f);
const ImVec4 kOutColor(0.9f, 0.25f, 0.25f, 1.0f);

void legend()
{
    ImGui::TextColored(kInColor, "IN");
    ImGui::SameLine();
    ImGui::TextColored(kOnColor, "ON");
    ImGui::SameLine();
    ImGui::TextColored(kOutColor, "OUT");
}

void sectionTitle(const char* text)
{
    ImGui::Spacing();
    ImGui::SeparatorText(text);
}

const char* opSymbol(csg::NodeType type)
{
    switch (type)
    {
    case csg::NodeType::Union: return "A u B";
    case csg::NodeType::Intersection: return "A n B";
    case csg::NodeType::Difference: return "A - B";
    default: return "";
    }
}

// Files with the extension in the usual folders, for the file popups.
std::vector<std::string> findFiles(const char* ext)
{
    std::vector<std::string> out;
    for (const char* dir : { ".", "scenes", "model", "models" })
    {
        std::error_code ec;
        for (const auto& entry : std::filesystem::directory_iterator(dir, ec))
            if (entry.is_regular_file() && entry.path().extension() == ext)
                out.push_back(entry.path().lexically_normal().generic_string());
    }
    std::sort(out.begin(), out.end());
    return out;
}

// Modal with a path field and the matching files. Returns true on confirm.
bool filePopup(const char* title, const char* ext, char* path, size_t size, const char* action)
{
    bool confirmed = false;
    ImGui::SetNextWindowSize(ImVec2(32.0f * ImGui::GetFontSize(), 0.0f));
    if (!ImGui::BeginPopupModal(title, nullptr, ImGuiWindowFlags_NoSavedSettings))
        return false;
    ImGui::TextDisabled("Path (relative to the working directory):");
    ImGui::SetNextItemWidth(-1.0f);
    if (ImGui::IsWindowAppearing())
        ImGui::SetKeyboardFocusHere();
    confirmed |= ImGui::InputText("##path", path, size, ImGuiInputTextFlags_EnterReturnsTrue);

    std::vector<std::string> files = findFiles(ext);
    ImGui::BeginChild("files", ImVec2(0, 8.0f * ImGui::GetFontSize()), ImGuiChildFlags_Borders);
    for (const std::string& f : files)
    {
        if (ImGui::Selectable(f.c_str(), f == path, ImGuiSelectableFlags_AllowDoubleClick))
        {
            std::snprintf(path, size, "%s", f.c_str());
            if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left))
                confirmed = true;
        }
    }
    if (files.empty())
        ImGui::TextDisabled("No %s files in ., scenes/, model/", ext);
    ImGui::EndChild();

    confirmed |= ImGui::Button(action, ImVec2(8.0f * ImGui::GetFontSize(), 0));
    ImGui::SameLine();
    bool cancel = ImGui::Button("Cancel", ImVec2(8.0f * ImGui::GetFontSize(), 0)) || ImGui::IsKeyPressed(ImGuiKey_Escape);
    if (confirmed || cancel)
        ImGui::CloseCurrentPopup();
    ImGui::EndPopup();
    return confirmed;
}

} // namespace

void App::setStatus(const std::string& message, bool isError)
{
    m_status = message;
    m_statusError = isError;
    m_statusTime = ImGui::GetTime();
    std::fprintf(isError ? stderr : stdout, "%s\n", message.c_str());
}

bool App::openScene(const std::string& path)
{
    std::string error;
    if (!csg::loadScene(m_scene, path, error))
    {
        setStatus("Open failed: " + error, true);
        return false;
    }
    m_selection.clear();
    std::snprintf(m_scenePath, sizeof(m_scenePath), "%s", path.c_str());
    frameSelection();
    setStatus("Opened " + path);
    return true;
}

void App::saveSceneTo(const std::string& path)
{
    std::error_code ec;
    std::filesystem::path dir = std::filesystem::path(path).parent_path();
    if (!dir.empty())
        std::filesystem::create_directories(dir, ec);
    std::string error;
    if (csg::saveScene(m_scene, path, error))
        setStatus("Saved " + path);
    else
        setStatus("Save failed: " + error, true);
}

void App::importMesh(const std::string& path)
{
    std::string error;
    csg::Node* node = m_scene.addMesh(path, error);
    if (!node)
    {
        setStatus("Import failed: " + error, true);
        return;
    }
    node->position = csg::Track<glm::vec3>(m_camera.target);
    select(node->id, false);
    const csg::Mesh& mesh = *node->mesh;
    char buf[256];
    std::snprintf(buf, sizeof(buf), "Imported %s: %zu triangles, %s", path.c_str(), mesh.triangleCount(),
                  mesh.closed() ? "closed surface" : "NOT closed (solid may be wrong)");
    setStatus(buf, !mesh.closed());
}

void App::drawUi()
{
    ImGuiIO& io = ImGui::GetIO();

    if (!io.WantTextInput)
    {
        if (ImGui::IsKeyPressed(ImGuiKey_Space, false))
            m_playing = !m_playing;
        if (ImGui::IsKeyPressed(ImGuiKey_F, false))
            frameSelection();
        if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_S, false))
            m_openPopup = "Save scene";
        if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_O, false))
            m_openPopup = "Open scene";
        if (ImGui::IsKeyPressed(ImGuiKey_Delete, false))
            if (csg::Node* node = primary(); node && m_scene.isRoot(node->id))
                m_scene.removeRoot(node->id);
    }

    drawMenuBar();

    // File dialogs (opened from the menu and the Primitives section).
    for (const char* name : { "Save scene", "Open scene", "Import STL mesh" })
        if (m_openPopup == name)
            ImGui::OpenPopup(name);
    m_openPopup.clear();
    if (filePopup("Save scene", ".txt", m_scenePath, sizeof(m_scenePath), "Save"))
        saveSceneTo(m_scenePath);
    if (filePopup("Open scene", ".txt", m_scenePath, sizeof(m_scenePath), "Open"))
        openScene(m_scenePath);
    if (filePopup("Import STL mesh", ".stl", m_meshPath, sizeof(m_meshPath), "Import"))
        importMesh(m_meshPath);
    drawTreePanel();
    drawViewport();
    drawPropertiesPanel();
    drawAnalysisPanel();
    drawTimeline();

    if (m_showImGuiDemo)
        ImGui::ShowDemoWindow(&m_showImGuiDemo);
}

void App::drawMenuBar()
{
    if (!ImGui::BeginMainMenuBar())
        return;
    if (ImGui::BeginMenu("File"))
    {
        if (ImGui::MenuItem("New scene"))
        {
            m_scene.clear();
            m_selection.clear();
        }
        if (ImGui::MenuItem("Load demo scene"))
        {
            m_scene.loadDemo();
            m_selection.clear();
        }
        ImGui::Separator();
        if (ImGui::MenuItem("Open scene (.txt)...", "Ctrl+O"))
            m_openPopup = "Open scene";
        if (ImGui::MenuItem("Save scene (.txt)...", "Ctrl+S"))
            m_openPopup = "Save scene";
        ImGui::Separator();
        if (ImGui::MenuItem("Import STL mesh..."))
            m_openPopup = "Import STL mesh";
        ImGui::Separator();
        if (ImGui::MenuItem("Quit"))
            glfwSetWindowShouldClose(m_window, 1);
        ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("View"))
    {
        if (ImGui::MenuItem("Frame selection", "F"))
            frameSelection();
        if (ImGui::MenuItem("Reset camera"))
            m_camera = render::Camera{};
        if (ImGui::MenuItem("Reload shaders"))
        {
            m_ray.reload();
            m_wire.reload();
        }
        ImGui::MenuItem("ImGui demo", nullptr, &m_showImGuiDemo);
        ImGui::EndMenu();
    }
    ImGui::EndMainMenuBar();
}

// ---------------------------------------------------------------- left: tree generators and operators

void App::drawTreePanel()
{
    const ImGuiIO& io = ImGui::GetIO();
    float fs = ImGui::GetFontSize();
    float top = ImGui::GetFrameHeight();
    float timelineH = 6.5f * fs;
    ImGui::SetNextWindowPos(ImVec2(0.0f, top));
    ImGui::SetNextWindowSize(ImVec2(20.0f * fs, io.DisplaySize.y - top - timelineH));
    ImGui::Begin("Trees", nullptr, kPanelFlags);

    sectionTitle("Primitives");
    const csg::PrimType types[] = { csg::PrimType::Block, csg::PrimType::Sphere, csg::PrimType::Cylinder,
                                    csg::PrimType::Cone };
    for (int i = 0; i < 4; ++i)
    {
        if (i > 0)
            ImGui::SameLine();
        if (ImGui::Button(csg::primTypeName(types[i])))
        {
            csg::Node* node = m_scene.addPrimitive(types[i]);
            node->position = csg::Track<glm::vec3>(m_camera.target);
            select(node->id, false);
        }
    }
    if (ImGui::Button("Mesh (STL)..."))
        m_openPopup = "Import STL mesh";
    ImGui::SameLine();
    ImGui::TextDisabled("closed triangle mesh as a solid");

    // Set operators combine two whole trees.
    sectionTitle("Set operations");
    csg::Node* a = nullptr;
    csg::Node* b = nullptr;
    // Always two lines, so the buttons below don't move as the selection changes.
    if (m_selection.size() == 2 && m_scene.isRoot(m_selection[0]) && m_scene.isRoot(m_selection[1]))
    {
        a = m_scene.find(m_selection[0]);
        b = m_scene.find(m_selection[1]);
        ImGui::Text("A: %s", a->name.c_str());
        ImGui::Text("B: %s", b->name.c_str());
    }
    else
    {
        ImGui::TextDisabled("A: -   Ctrl+click two trees,");
        ImGui::TextDisabled("B: -   A first, then B");
    }
    ImGui::BeginDisabled(!a);
    const std::pair<const char*, csg::NodeType> ops[] = { { "Union", csg::NodeType::Union },
                                                          { "Intersection", csg::NodeType::Intersection },
                                                          { "Difference", csg::NodeType::Difference } };
    for (int i = 0; i < 3; ++i)
    {
        if (i > 0)
            ImGui::SameLine();
        if (ImGui::Button(ops[i].first) && a && b)
            if (csg::Node* node = m_scene.combine(a->id, b->id, ops[i].second))
                select(node->id, false);
    }
    ImGui::EndDisabled();

    // Ungroup and Delete destroy nodes, so the selection is re-read after them.
    csg::Node* sel = primary();
    bool selIsRoot = sel && m_scene.isRoot(sel->id);
    auto refresh = [&] {
        sel = primary();
        selIsRoot = sel && m_scene.isRoot(sel->id);
    };
    bool canUngroup = selIsRoot && !sel->isPrimitive();
    ImGui::BeginDisabled(!canUngroup);
    if (ImGui::Button("Ungroup") && canUngroup)
    {
        int first = sel->a->id;
        m_scene.ungroup(sel->id);
        m_selection = { first };
        refresh();
    }
    ImGui::EndDisabled();
    ImGui::SameLine();
    bool canDelete = selIsRoot;
    ImGui::BeginDisabled(!canDelete);
    if (ImGui::Button("Delete") && canDelete)
    {
        m_scene.removeRoot(sel->id);
        m_selection.clear();
        refresh();
    }
    ImGui::EndDisabled();
    ImGui::SameLine();
    bool canSelectRoot = sel && !selIsRoot;
    ImGui::BeginDisabled(!canSelectRoot);
    if (ImGui::Button("Select root") && canSelectRoot)
    {
        m_selection = { primaryRoot()->id };
        refresh();
    }
    ImGui::EndDisabled();

    // Geometric operators act on the selected (sub)tree; with global modeling they
    // are pushed down to every primitive in it.
    sectionTitle("Geometric operations");
    if (!sel)
        ImGui::TextDisabled("Select a tree or node");
    else
        ImGui::Text("On: %s", sel->name.c_str());
    ImGui::BeginDisabled(!sel);
    glm::vec3 pivot = sel ? m_scene.bounds(sel, m_scene.time).center() : glm::vec3(0.0f);
    ImGui::PushItemWidth(-6.0f * fs);
    ImGui::DragFloat3("##translate", &m_translate.x, 0.01f);
    ImGui::SameLine();
    if (ImGui::Button("Translate", ImVec2(-1, 0)))
        m_scene.translate(sel, m_translate);
    ImGui::DragFloat3("##axis", &m_rotateAxis.x, 0.01f);
    ImGui::SameLine();
    ImGui::TextUnformatted("Axis");
    ImGui::DragFloat("##angle", &m_rotateDegrees, 0.5f, -360.0f, 360.0f, "%.1f deg");
    ImGui::SameLine();
    if (ImGui::Button("Rotate", ImVec2(-1, 0)) && glm::length(m_rotateAxis) > 1e-6f)
        m_scene.rotate(sel, glm::angleAxis(glm::radians(m_rotateDegrees), glm::normalize(m_rotateAxis)), pivot);
    ImGui::DragFloat("##scale", &m_scaleFactor, 0.01f, 0.05f, 20.0f, "x %.2f");
    ImGui::SameLine();
    if (ImGui::Button("Scale", ImVec2(-1, 0)))
        m_scene.scale(sel, m_scaleFactor, pivot);
    ImGui::PopItemWidth();
    ImGui::EndDisabled();
    ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
    ImGui::TextWrapped("Pivot: tree center. Scale changes the size parameters, not a matrix.");
    ImGui::PopStyleColor();

    sectionTitle("Trees");
    ImGui::BeginChild("outliner", ImVec2(0, 0), ImGuiChildFlags_Borders);
    for (const auto& root : m_scene.roots)
        drawOutlinerNode(root.get(), nullptr);
    if (m_scene.roots.empty())
        ImGui::TextDisabled("Empty scene: add a primitive");
    ImGui::EndChild();

    ImGui::End();
}

void App::drawOutlinerNode(csg::Node* node, const char* role)
{
    ImGuiTreeNodeFlags flags = ImGuiTreeNodeFlags_SpanAvailWidth | ImGuiTreeNodeFlags_OpenOnArrow |
                               ImGuiTreeNodeFlags_DefaultOpen;
    if (node->isPrimitive())
        flags |= ImGuiTreeNodeFlags_Leaf | ImGuiTreeNodeFlags_NoTreePushOnOpen;
    if (isSelected(node->id))
        flags |= ImGuiTreeNodeFlags_Selected;

    char label[160];
    if (node->isPrimitive())
        std::snprintf(label, sizeof(label), "%s%s", role ? role : "", node->name.c_str());
    else
        std::snprintf(label, sizeof(label), "%s%s  (%s)", role ? role : "", node->name.c_str(), opSymbol(node->type));

    ImGui::PushID(node->id);
    bool open = ImGui::TreeNodeEx("node", flags, "%s", label);
    if (ImGui::IsItemClicked() && !ImGui::IsItemToggledOpen())
        select(node->id, ImGui::GetIO().KeyCtrl);
    if (node->isPrimitive() && node->position.animated() + node->rotation.animated() + node->size.animated())
    {
        ImGui::SameLine();
        ImGui::TextDisabled("(anim)");
    }
    if (open && !node->isPrimitive())
    {
        drawOutlinerNode(node->a.get(), "A: ");
        drawOutlinerNode(node->b.get(), "B: ");
        ImGui::TreePop();
    }
    ImGui::PopID();
}

// ---------------------------------------------------------------- right: properties

void App::drawPropertiesPanel()
{
    const ImGuiIO& io = ImGui::GetIO();
    float fs = ImGui::GetFontSize();
    float top = ImGui::GetFrameHeight();
    float width = 23.0f * fs;
    float height = (io.DisplaySize.y - top - 6.5f * fs) * 0.5f;
    ImGui::SetNextWindowPos(ImVec2(io.DisplaySize.x - width, top));
    ImGui::SetNextWindowSize(ImVec2(width, height));
    ImGui::Begin("Properties", nullptr, kPanelFlags);

    csg::Node* node = primary();
    if (!node)
    {
        ImGui::TextDisabled("Nothing selected");
        ImGui::End();
        return;
    }

    char name[64];
    std::snprintf(name, sizeof(name), "%s", node->name.c_str());
    if (ImGui::InputText("Name", name, sizeof(name)))
        node->name = name;

    const float t = m_scene.time;
    const bool key = m_scene.autoKey;

    if (!node->isPrimitive())
    {
        int type = int(node->type) - 1;
        if (ImGui::Combo("Operation", &type, "Union\0Intersection\0Difference\0"))
            node->type = csg::NodeType(type + 1);
        if (ImGui::Button("Swap A and B"))
            std::swap(node->a, node->b);
        ImGui::Text("A: %s", node->a->name.c_str());
        ImGui::Text("B: %s", node->b->name.c_str());
        ImGui::TextDisabled("Transform the whole tree with the\ngeometric operations on the left.");
    }
    else
    {
        ImGui::Text("Type: %s", csg::primTypeName(node->prim));
        if (node->prim == csg::PrimType::Mesh && node->mesh)
        {
            const csg::Mesh& mesh = *node->mesh;
            ImGui::TextDisabled("File: %s", node->meshPath.c_str());
            ImGui::TextDisabled("%zu triangles, BVH %zu nodes", mesh.triangleCount(), mesh.nodes().size());
            if (mesh.closed())
                ImGui::TextColored(kInColor, "Closed, consistently oriented surface");
            else
                ImGui::TextColored(kOutColor, "Open: %d edges, flipped: %d", mesh.openEdges(), mesh.flippedEdges());
        }

        glm::vec3 pos = node->position.value(t);
        if (ImGui::DragFloat3("Position", &pos.x, 0.01f))
            node->position.set(t, pos, key);

        glm::vec3 euler = glm::degrees(glm::eulerAngles(node->rotation.value(t)));
        if (ImGui::DragFloat3("Rotation", &euler.x, 0.5f))
            node->rotation.set(t, glm::quat(glm::radians(euler)), key);

        glm::vec3 size = node->size.value(t);
        bool sizeChanged = false;
        switch (node->prim)
        {
        case csg::PrimType::Block:
            sizeChanged = ImGui::DragFloat3("Size (W H D)", &size.x, 0.01f, 0.01f, 100.0f);
            break;
        case csg::PrimType::Sphere:
            sizeChanged = ImGui::DragFloat("Radius", &size.x, 0.01f, 0.01f, 100.0f);
            break;
        case csg::PrimType::Mesh:
            sizeChanged = ImGui::DragFloat("Size (largest extent)", &size.x, 0.01f, 0.01f, 100.0f);
            break;
        default:
            sizeChanged = ImGui::DragFloat("Radius", &size.x, 0.01f, 0.01f, 100.0f);
            sizeChanged |= ImGui::DragFloat("Height", &size.y, 0.01f, 0.01f, 100.0f);
            break;
        }
        if (sizeChanged)
            node->size.set(t, glm::max(size, glm::vec3(0.01f)), key);

        ImGui::ColorEdit3("Color", &node->color.x);
        ImGui::SliderFloat("Reflectivity", &node->reflectivity, 0.0f, 1.0f);

        ImGui::Checkbox("Limited lifetime", &node->limitedLife);
        if (node->limitedLife)
            ImGui::DragFloatRange2("Exists", &node->visibleFrom, &node->visibleTo, 0.01f, 0.0f, m_scene.duration,
                                   "from %.2f s", "to %.2f s");

        csg::Primitive p = node->evaluate(t);
        ImGui::TextDisabled("Primitive: V = %.4f   A = %.4f", p.volume(), p.area());
    }

    sectionTitle("Animation");
    int keyCount = 0;
    bool keyedHere = false;
    csg::Scene::forEachPrimitive(static_cast<const csg::Node*>(node), [&](const csg::Node& n) {
        keyCount += int(n.position.keys().size() + n.rotation.keys().size() + n.size.keys().size()) - 3;
        keyedHere |= n.position.hasKeyAt(t) || n.rotation.hasKeyAt(t) || n.size.hasKeyAt(t);
    });
    ImGui::Text("Extra keys: %d   %s", keyCount, keyedHere ? "(keyed at this time)" : "");
    if (ImGui::Button("Key at t"))
        m_scene.keyAll(node);
    ImGui::SameLine();
    if (ImGui::Button("Delete keys at t"))
        m_scene.removeKeys(node);
    ImGui::SameLine();
    if (ImGui::Button("Clear"))
        m_scene.clearAnimation(node);

    ImGui::End();
}

// ---------------------------------------------------------------- right: analysis, SMC probe, render settings

void App::drawAnalysisPanel()
{
    const ImGuiIO& io = ImGui::GetIO();
    float fs = ImGui::GetFontSize();
    float top = ImGui::GetFrameHeight();
    float width = 23.0f * fs;
    float height = (io.DisplaySize.y - top - 6.5f * fs) * 0.5f;
    ImGui::SetNextWindowPos(ImVec2(io.DisplaySize.x - width, top + height));
    ImGui::SetNextWindowSize(ImVec2(width, height));
    ImGui::Begin("Analysis", nullptr, kPanelFlags);

    if (ImGui::CollapsingHeader("Volume and area", ImGuiTreeNodeFlags_DefaultOpen))
    {
        csg::Node* node = primary();
        if (!node)
            ImGui::TextDisabled("Select a tree");
        else
            ImGui::Text("Tree: %s", node->name.c_str());

        ImGui::SliderInt("Rays/axis", &m_analysis.volumeRes, 32, 512);
        ImGui::SliderInt("Samples", &m_analysis.areaSamples, 1000, 100000);
        ImGui::BeginDisabled(!node);
        if (ImGui::Button("Compute"))
            computeAnalysis(false);
        ImGui::SameLine();
        ImGui::Checkbox("Live", &m_analysis.live);
        ImGui::SameLine();
        if (ImGui::Button("+ Hypervolume"))
            computeAnalysis(true);
        ImGui::EndDisabled();

        if (node && m_analysis.nodeId == node->id)
        {
            ImGui::Text("Volume:       %.5f", m_analysis.volume);
            ImGui::Text("Surface area: %.5f", m_analysis.area);
            ImGui::TextDisabled("at t = %.2f s, %.1f ms", m_analysis.time, m_analysis.milliseconds);
            if (m_analysis.hypervolume >= 0.0)
                ImGui::Text("Hypervolume (vol x s over 0..%.1f s): %.5f", m_scene.duration, m_analysis.hypervolume);
        }
    }

    if (ImGui::CollapsingHeader("SMC probe line"))
    {
        ImGui::Checkbox("Show probe", &m_probe.enabled);
        ImGui::DragFloat3("A", &m_probe.a.x, 0.01f);
        ImGui::DragFloat3("B", &m_probe.b.x, 0.01f);
        const csg::Node* tree = probeTree();
        ImGui::TextDisabled("Against: %s", tree ? tree->name.c_str() : "-");
        legend();
        if (m_probe.enabled && ImGui::BeginTable("probe", 3, ImGuiTableFlags_Borders | ImGuiTableFlags_SizingStretchProp))
        {
            ImGui::TableSetupColumn("Class");
            ImGui::TableSetupColumn("from t");
            ImGui::TableSetupColumn("to t");
            ImGui::TableHeadersRow();
            auto row = [](const ImVec4& color, const char* cls, float t0, float t1) {
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::TextColored(color, "%s", cls);
                ImGui::TableNextColumn();
                ImGui::Text("%.4f", t0);
                ImGui::TableNextColumn();
                ImGui::Text("%.4f", t1);
            };
            float t = 0.0f;
            for (const csg::Span& s : m_probe.spans)
            {
                if (s.t0 > t)
                    row(kOutColor, "OUT", t, s.t0);
                row(kOnColor, "ON", s.t0, s.t0);
                row(kInColor, "IN", s.t0, s.t1);
                row(kOnColor, "ON", s.t1, s.t1);
                t = s.t1;
            }
            if (t < 1.0f)
                row(kOutColor, "OUT", t, 1.0f);
            ImGui::EndTable();
        }
    }

    if (ImGui::CollapsingHeader("Render settings"))
    {
        ImGui::SliderInt("Bounces", &m_settings.bounces, 0, 6);
        ImGui::Checkbox("Ground plane", &m_settings.ground);
        ImGui::SameLine();
        ImGui::DragFloat("##groundY", &m_settings.groundY, 0.01f, -50.0f, 50.0f, "y = %.2f");
        ImGui::DragFloat3("Light dir", &m_settings.lightDir.x, 0.01f, -1.0f, 1.0f);
        ImGui::SliderFloat("SMC opacity", &m_settings.smcAlpha, 0.05f, 1.0f);
    }

    ImGui::End();
}

// ---------------------------------------------------------------- bottom: timeline (4th dimension)

void App::drawTimeline()
{
    const ImGuiIO& io = ImGui::GetIO();
    float fs = ImGui::GetFontSize();
    float height = 6.5f * fs;
    ImGui::SetNextWindowPos(ImVec2(0.0f, io.DisplaySize.y - height));
    ImGui::SetNextWindowSize(ImVec2(io.DisplaySize.x, height));
    ImGui::Begin("Timeline", nullptr, kPanelFlags);

    if (ImGui::Button(m_playing ? "Pause" : "Play", ImVec2(5.0f * fs, 0)))
        m_playing = !m_playing;
    ImGui::SameLine();
    if (ImGui::Button("|<"))
        m_scene.time = 0.0f;
    ImGui::SameLine();
    ImGui::Checkbox("Loop", &m_loop);
    ImGui::SameLine();
    ImGui::SetNextItemWidth(6.0f * fs);
    ImGui::DragFloat("Speed", &m_speed, 0.01f, 0.05f, 10.0f, "%.2fx");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(6.0f * fs);
    if (ImGui::DragFloat("Duration", &m_scene.duration, 0.05f, 0.1f, 600.0f, "%.1f s"))
        m_scene.time = std::min(m_scene.time, m_scene.duration);
    ImGui::SameLine();
    ImGui::Checkbox("Auto-key edits", &m_scene.autoKey);
    ImGui::SameLine();
    ImGui::TextDisabled("(Space: play/pause)");

    ImGui::SetNextItemWidth(-1.0f);
    ImGui::SliderFloat("##time", &m_scene.time, 0.0f, m_scene.duration, "t = %.2f s");

    // Key markers of the selected tree (all trees if nothing is selected).
    std::set<float> times;
    auto collect = [&](const csg::Node& n) {
        for (const auto& k : n.position.keys()) times.insert(k.time);
        for (const auto& k : n.rotation.keys()) times.insert(k.time);
        for (const auto& k : n.size.keys()) times.insert(k.time);
    };
    if (const csg::Node* node = primary())
        csg::Scene::forEachPrimitive(node, collect);
    else
        for (const auto& root : m_scene.roots)
            csg::Scene::forEachPrimitive(static_cast<const csg::Node*>(root.get()), collect);

    ImVec2 min = ImGui::GetItemRectMin(), max = ImGui::GetItemRectMax();
    float grab = ImGui::GetStyle().GrabMinSize * 0.5f + ImGui::GetStyle().FramePadding.x * 0.5f;
    ImDrawList* draw = ImGui::GetWindowDrawList();
    for (float kt : times)
    {
        if (kt > m_scene.duration || (times.size() == 1 && kt == 0.0f))
            continue;
        float x = min.x + grab + (max.x - min.x - 2.0f * grab) * (kt / m_scene.duration);
        float y = max.y + 0.35f * fs;
        float r = 0.3f * fs;
        draw->AddQuadFilled(ImVec2(x, y - r), ImVec2(x + r, y), ImVec2(x, y + r), ImVec2(x - r, y),
                            IM_COL32(255, 200, 60, 255));
    }

    ImGui::End();
}

// ---------------------------------------------------------------- center: viewport

void App::drawViewport()
{
    const ImGuiIO& io = ImGui::GetIO();
    float fs = ImGui::GetFontSize();
    float top = ImGui::GetFrameHeight();
    float left = 20.0f * fs, right = 23.0f * fs, bottom = 6.5f * fs;
    ImGui::SetNextWindowPos(ImVec2(left, top));
    ImGui::SetNextWindowSize(ImVec2(io.DisplaySize.x - left - right, io.DisplaySize.y - top - bottom));
    ImGui::Begin("Viewport", nullptr, kPanelFlags | ImGuiWindowFlags_NoScrollbar);

    int mode = int(m_settings.mode);
    ImGui::RadioButton("Ray casting", &mode, int(render::RenderMode::RayCast));
    ImGui::SameLine();
    ImGui::RadioButton("Ray tracing", &mode, int(render::RenderMode::RayTrace));
    ImGui::SameLine();
    ImGui::RadioButton("SMC", &mode, int(render::RenderMode::Smc));
    ImGui::SameLine();
    ImGui::RadioButton("Wireframe", &mode, int(render::RenderMode::Wireframe));
    m_settings.mode = render::RenderMode(mode);
    ImGui::SameLine();
    ImGui::BeginDisabled(m_settings.mode == render::RenderMode::Wireframe);
    ImGui::Checkbox("Edges (SMC)", &m_wireOverlay);
    ImGui::EndDisabled();

    ImVec2 avail = ImGui::GetContentRegionAvail();
    m_viewportWidth = std::max(1, int(avail.x * io.DisplayFramebufferScale.x));
    m_viewportHeight = std::max(1, int(avail.y * io.DisplayFramebufferScale.y));

    ImVec2 pos = ImGui::GetCursorScreenPos();
    ImGui::InvisibleButton("viewport", ImVec2(std::max(avail.x, 1.0f), std::max(avail.y, 1.0f)),
                           ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight |
                               ImGuiButtonFlags_MouseButtonMiddle);
    ImDrawList* draw = ImGui::GetWindowDrawList();
    draw->AddImage((ImTextureID)(intptr_t)m_viewport.texture(), pos, ImVec2(pos.x + avail.x, pos.y + avail.y),
                   ImVec2(0, 1), ImVec2(1, 0));

    // Orbit with left drag, pan with right/middle drag, zoom with the wheel; click to pick.
    if (ImGui::IsItemActive())
    {
        if (ImGui::IsMouseDragging(ImGuiMouseButton_Left))
            m_camera.orbit(io.MouseDelta.x, io.MouseDelta.y);
        if (ImGui::IsMouseDragging(ImGuiMouseButton_Right) || ImGui::IsMouseDragging(ImGuiMouseButton_Middle))
            m_camera.pan(io.MouseDelta.x, io.MouseDelta.y);
    }
    if (ImGui::IsItemHovered() && io.MouseWheel != 0.0f)
        m_camera.zoom(io.MouseWheel);
    if (ImGui::IsItemDeactivated() && ImGui::IsMouseReleased(ImGuiMouseButton_Left) &&
        io.MouseDragMaxDistanceSqr[ImGuiMouseButton_Left] < 16.0f)
    {
        // pick() works in framebuffer pixels
        float sx = io.DisplayFramebufferScale.x, sy = io.DisplayFramebufferScale.y;
        pick(io.MousePos.x * sx, io.MousePos.y * sy, pos.x * sx, pos.y * sy, io.KeyCtrl, io.KeyShift);
    }

    // Overlays
    float pad = 0.5f * fs;
    ImVec2 textPos(pos.x + pad, pos.y + pad);
    auto overlay = [&](ImU32 color, const char* text) {
        draw->AddText(textPos, color, text);
        textPos.y += fs * 1.2f;
    };
    if (!m_ray.error().empty())
        overlay(IM_COL32(255, 90, 90, 255), m_ray.error().c_str());
    if (m_gpuLimitHit)
        overlay(IM_COL32(255, 90, 90, 255), m_ray.limitError().c_str());
    if (!m_status.empty() && ImGui::GetTime() - m_statusTime < 6.0)
        overlay(m_statusError ? IM_COL32(255, 90, 90, 255) : IM_COL32(140, 230, 140, 255), m_status.c_str());
    if (m_settings.mode == render::RenderMode::Smc)
        overlay(IM_COL32(220, 220, 220, 255),
                primaryRoot() ? "SMC of every primitive surface against the selected tree"
                              : "SMC against all trees (select a tree to isolate it)");
    if (m_settings.mode == render::RenderMode::Wireframe || m_wireOverlay)
        overlay(IM_COL32(220, 220, 220, 255), "Edges classified against their own tree");

    if (m_settings.mode == render::RenderMode::Smc || m_settings.mode == render::RenderMode::Wireframe ||
        m_wireOverlay || m_probe.enabled)
    {
        ImVec2 lp(pos.x + pad, pos.y + avail.y - pad - fs);
        draw->AddText(lp, ImGui::GetColorU32(kInColor), "IN");
        draw->AddText(ImVec2(lp.x + 2.0f * fs, lp.y), ImGui::GetColorU32(kOnColor), "ON");
        draw->AddText(ImVec2(lp.x + 4.0f * fs, lp.y), ImGui::GetColorU32(kOutColor), "OUT");
    }

    char fps[48];
    std::snprintf(fps, sizeof(fps), "%.0f fps  %.1f ms", io.Framerate, 1000.0f / io.Framerate);
    ImVec2 fs2 = ImGui::CalcTextSize(fps);
    draw->AddText(ImVec2(pos.x + avail.x - fs2.x - pad, pos.y + pad), IM_COL32(200, 200, 200, 160), fps);

    const char* help = "LMB orbit | RMB pan | wheel zoom | click select (Ctrl add, Shift tree) | F frame";
    ImVec2 hs = ImGui::CalcTextSize(help);
    draw->AddText(ImVec2(pos.x + avail.x - hs.x - pad, pos.y + avail.y - pad - fs), IM_COL32(200, 200, 200, 160), help);

    ImGui::End();
}

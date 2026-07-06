//
// 地图目录分层布局工具
//
// 对齐 dexforce 地图管理规范：<maps_root>/<map_name>/{slam,assets,versions}/
// 底图（点云 tiles / global.pcd / index.txt / map.png / map.yaml）统一存放在
// <map_name>/slam/ 子目录；assets/ 与 versions/ 归其他组件管理，本包不触碰。
// 底图保存时仅清空/重建 slam/ 子目录，保证不影响同图的资产与版本数据。
//

#ifndef LIGHTNING_MAP_LAYOUT_H
#define LIGHTNING_MAP_LAYOUT_H

#include <cstdlib>
#include <filesystem>
#include <string>

namespace lightning {

/// 默认地图根目录：$HOME/.dexe/data/maps（与 launch 的 default_data_dir 一致）
inline std::string DefaultMapsRoot() {
    const char* home = std::getenv("HOME");
    if (home == nullptr || home[0] == '\0') {
        return "./data";
    }
    return std::string(home) + "/.dexe/data/maps";
}

/// 解析地图根目录：空串取默认根目录；"~" 前缀展开为 $HOME
inline std::string ResolveMapsRoot(const std::string& folder) {
    if (folder.empty()) {
        return DefaultMapsRoot();
    }
    if (folder[0] == '~') {
        const char* home = std::getenv("HOME");
        if (home != nullptr && home[0] != '\0') {
            return std::string(home) + folder.substr(1);
        }
    }
    return folder;
}

/// 底图保存目录（写侧固定分层布局）：<folder>/<name>/slam
inline std::string MapSlamDir(const std::string& folder, const std::string& name) {
    return folder + "/" + name + "/slam";
}

/// 底图读取目录（读侧兼容旧扁平布局）：优先 <name>/slam/，回退 <name>/
inline std::string ResolveMapDir(const std::string& folder, const std::string& name) {
    namespace fs = std::filesystem;
    const std::string layered = MapSlamDir(folder, name);
    if (fs::exists(fs::path(layered) / "global.pcd")) {
        return layered;
    }
    const std::string legacy = folder + "/" + name;
    if (fs::exists(fs::path(legacy) / "global.pcd")) {
        return legacy;
    }
    return layered;  // 两者都不存在时返回新布局路径，由调用方报错
}

/// 从底图路径提取地图名（兼容尾随 '/' 与 slam/ 子目录）
inline std::string MapNameFromPath(const std::string& map_path) {
    namespace fs = std::filesystem;
    fs::path p(map_path);
    std::string name = p.filename().string();
    if (name.empty()) {
        p = p.parent_path();
        name = p.filename().string();
    }
    if (name == "slam") {
        name = p.parent_path().filename().string();
    }
    return name;
}

}  // namespace lightning

#endif  // LIGHTNING_MAP_LAYOUT_H

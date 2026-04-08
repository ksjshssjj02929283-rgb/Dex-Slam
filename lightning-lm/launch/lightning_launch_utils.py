import os


LIDAR_CONFIG_MAP = {
    'airy': 'default_airy.yaml',
    'livox': 'default_livox.yaml',
    'vbr': 'default_vbr.yaml',
}


def default_data_dir():
    return os.path.join(os.path.expanduser('~'), '.dexe', 'data')


def resolve_config_file(pkg_dir, lidar_type):
    normalized_lidar_type = lidar_type.strip().lower()
    if normalized_lidar_type not in LIDAR_CONFIG_MAP:
        supported = ', '.join(sorted(LIDAR_CONFIG_MAP.keys()))
        raise RuntimeError(
            f'Unsupported lidar_type: {lidar_type}. Supported values: {supported}'
        )
    return os.path.join(pkg_dir, 'config', LIDAR_CONFIG_MAP[normalized_lidar_type])
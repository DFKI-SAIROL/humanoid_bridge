"""Dashboard labels only; Rerun's built-in menus keep their own language."""
ZH = {
    'H1-2 · Scene': 'H1-2 · 三维场景', 'Head · RGB': '头部 · 彩色图像',
    'Left hand · RGB': '左手 · 彩色图像', 'Right hand · RGB': '右手 · 彩色图像',
    'Head · Depth': '头部 · 深度图', 'Observe': '主界面', 'Debug': '调试',
    'Connection': '连接状态', 'Joints': '关节', 'Hands': '双手反馈',
    'Joint values': '关节数值', 'Frames': '坐标变换', 'Sensors': '传感器',
    'position': '位置', 'velocity': '速度', 'torque': '力矩',
    'left_arm': '左臂', 'right_arm': '右臂', 'left_leg': '左腿', 'right_leg': '右腿',
    'waist': '腰部', 'Hand feedback': '双手原始反馈', 'Hand targets': '双手归一化目标',
    'Roll': '横滚 Roll', 'Pitch': '俯仰 Pitch', 'Yaw': '航向 Yaw',
    'Roll / Pitch · deg': '横滚 / 俯仰 · 度', 'Yaw · deg': '航向 · 度',
    'Quaternion · xyzw': '四元数 · xyzw', 'IMU status': 'IMU 状态',
}


def translator(settings):
    return lambda label: ZH.get(label, label) if settings['layout']['language'] == 'zh' else label

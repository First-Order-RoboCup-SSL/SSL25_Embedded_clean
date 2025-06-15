import struct

def calculate_can_command(can_id, target_speed):
    """
    计算速度模式下的CAN指令。

    :param can_id: 电机的CAN ID (0-127范围内的整数)
    :param target_speed: 目标速度，单位 rad/s
    :return: 帧ID和数据字节列表
    """
    if not (0 <= can_id <= 127):
        raise ValueError("CAN ID必须在0到127之间")

    # 计算帧ID
    frame_id = 0x200 + can_id

    # 将目标速度编码为浮点型，低字节在前（小端格式）
    speed_bytes = struct.pack('<f', target_speed)

    # 提取数据字节
    data = list(speed_bytes)

    return frame_id, data

# 示例用法
if __name__ == "__main__":
    can_id = 5  # 电机的CAN ID
    target_speed = 6.12  # 目标速度，单位为rad/s（等于60rpm）

    frame_id, data = calculate_can_command(can_id, target_speed)

    # 打印结果
    print(f"帧ID: 0x{frame_id:X}")
    print("数据:", [f"0x{byte:02X}" for byte in data])

import rclpy
from rclpy.node import Node
from dsr_msgs2.srv import DrlStart
import textwrap

DRL_BASE_CODE = """
g_slaveid = 0
flag = 0
def modbus_set_slaveid(slaveid):
    global g_slaveid
    g_slaveid = slaveid
def modbus_fc06(address, value):
    global g_slaveid
    data = (g_slaveid).to_bytes(1, byteorder='big')
    data += (6).to_bytes(1, byteorder='big')
    data += (address).to_bytes(2, byteorder='big')
    data += (value).to_bytes(2, byteorder='big')
    return modbus_send_make(data)
def modbus_fc16(startaddress, cnt, valuelist):
    global g_slaveid
    data = (g_slaveid).to_bytes(1, byteorder='big')
    data += (16).to_bytes(1, byteorder='big')
    data += (startaddress).to_bytes(2, byteorder='big')
    data += (cnt).to_bytes(2, byteorder='big')
    data += (2 * cnt).to_bytes(1, byteorder='big')
    for i in range(0, cnt):
        data += (valuelist[i]).to_bytes(2, byteorder='big')
    return modbus_send_make(data)
def recv_check():
    size, val = flange_serial_read(0.1)
    if size > 0:
        return True, val
    else:
        tp_log("CRC Check Fail")
        return False, val
def gripper_move(stroke):
    flange_serial_write(modbus_fc16(282, 2, [stroke, 0]))
    wait(1.0) # 물리적 동작 시간을 충분히 기다려줍니다.

# ---- init serial & torque/current ----
while True:
    flange_serial_open(
        baudrate=57600,
        bytesize=DR_EIGHTBITS,
        parity=DR_PARITY_NONE,
        stopbits=DR_STOPBITS_ONE,
    )

    modbus_set_slaveid(1)

    # 256(40257) Torque enable
    # 275(40276) Goal Current
    # 282(40283) Goal Position

    flange_serial_write(modbus_fc06(256, 1))   # torque enable
    flag, val = recv_check()

    flange_serial_write(modbus_fc06(275, 400)) # goal current
    flag, val = recv_check()

    if flag is True:
        break

    flange_serial_close()
"""

class GripperController:
    def __init__(self, node: Node, namespace: str = "dsr01", robot_system: int = 0):
        self.node = node
        self.robot_system = robot_system
        self.cli = self.node.create_client(DrlStart, f"/{namespace}/drl/drl_start")

        self.node.get_logger().info(f"Waiting for service /{namespace}/drl/drl_start...")
        while not self.cli.wait_for_service(timeout_sec=2.0):
            self.node.get_logger().info("Service not available, waiting again...")
        self.node.get_logger().info("GripperController is ready.")

    def _send_drl_script(self, code: str) -> bool:
        req = DrlStart.Request()
        req.robot_system = self.robot_system
        req.code = code
        future = self.cli.call_async(req)
        
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        if future.result() is not None:
            return bool(future.result().success)
        else:
            self.node.get_logger().error(f"Service call failed: {future.exception()}")
            return False

    def initialize(self) -> bool:
        self.node.get_logger().info("Initializing gripper connection...")
        task_code = textwrap.dedent("""
            flange_serial_open(baudrate=57600, bytesize=DR_EIGHTBITS, parity=DR_PARITY_NONE, stopbits=DR_STOPBITS_ONE)
            modbus_set_slaveid(1)
            flange_serial_write(modbus_fc06(256, 1))
            recv_check()
            flange_serial_write(modbus_fc06(275, 400))
            recv_check()
        """)
        init_script = f"{DRL_BASE_CODE}\n{task_code}"
        success = self._send_drl_script(init_script)
        if success:
            self.node.get_logger().info("Gripper connection initialized successfully.")
        else:
            self.node.get_logger().error("Failed to initialize gripper connection.")
        return success

    def move(self, stroke: int) -> bool:
        self.node.get_logger().info(f"Moving gripper to stroke: {stroke}")
        task_code = textwrap.dedent(f"""
            gripper_move({stroke})
        """)
        move_script = f"{DRL_BASE_CODE}\n{task_code}"
        success = self._send_drl_script(move_script)
        if success:
            self.node.get_logger().info(f"Gripper move command for {stroke} sent successfully.")
        else:
            self.node.get_logger().error("Failed to send gripper move command.")
        return success

    def report_status_to_tool_do(
        self,
        tool_do_base: int,
        slave_id: int = 9,
        start_addr: int = 0x07D0,
        reg_count: int = 3,
    ) -> bool:
        read_cmd = (
            f"\\x{slave_id:02X}"
            f"\\x03"
            f"\\x{(start_addr >> 8) & 0xFF:02X}\\x{start_addr & 0xFF:02X}"
            f"\\x{(reg_count >> 8) & 0xFF:02X}\\x{reg_count & 0xFF:02X}"
        )
        task_code = textwrap.dedent(f"""
            def _set_bit(idx, val):
                if val not in (0, 1):
                    val = 0
                set_tool_digital_output(idx, val)

            flange_serial_write(modbus_send_make(b"{read_cmd}"))
            res, data = flange_serial_read()
            if res > 0 and data is not None and len(data) >= 4:
                status = data[3]
                g_sta = (status >> 4) & 0x03
                g_obj = (status >> 6) & 0x03

                _set_bit({tool_do_base}, g_sta & 0x01)
                _set_bit({tool_do_base + 1}, (g_sta >> 1) & 0x01)
                _set_bit({tool_do_base + 2}, g_obj & 0x01)
                _set_bit({tool_do_base + 3}, (g_obj >> 1) & 0x01)
            else:
                _set_bit({tool_do_base}, 0)
                _set_bit({tool_do_base + 1}, 0)
                _set_bit({tool_do_base + 2}, 0)
                _set_bit({tool_do_base + 3}, 0)
        """)
        report_script = f"{DRL_BASE_CODE}\n{task_code}"
        success = self._send_drl_script(report_script)
        if not success:
            self.node.get_logger().error("Failed to execute gripper status report script.")
        return success

    def report_rhp12rna_status_to_tool_do(
        self,
        tool_do_base: int,
        pos_open_th: int = 50,
        pos_close_th: int = 690,
        slave_id: int = 1,
        start_addr: int = 285,
        reg_count: int = 7,
    ) -> bool:
        if reg_count < 7:
            reg_count = reg_count
        read_cmd = (
            f"\\x{slave_id:02X}"
            f"\\x03"
            f"\\x{(start_addr >> 8) & 0xFF:02X}\\x{start_addr & 0xFF:02X}"
            f"\\x{(reg_count >> 8) & 0xFF:02X}\\x{reg_count & 0xFF:02X}"
        )
        task_code = textwrap.dedent(f"""
            def _set_bit(idx, val):
                if val not in (0, 1):
                    val = 0
                set_tool_digital_output(idx, val)

            def _get_reg(data, idx):
                base = 3 + (idx * 2)
                if data is None:
                    return 0
                try:
                    if len(data) < base + 2:
                        return 0
                    return (data[base] << 8) | data[base + 1]
                except Exception:
                    return 0

            flange_serial_write(modbus_send_make(b"{read_cmd}"))
            res, data = flange_serial_read()
            if res > 0 and data is not None:
                reg0 = _get_reg(data, 0)
                moving = reg0 & 0xFF
                moving_status = (reg0 >> 8) & 0xFF

                pos_lo = _get_reg(data, 5)
                pos_hi = _get_reg(data, 6)
                present_pos = (pos_hi << 16) | pos_lo

                is_open = 1 if present_pos <= {pos_open_th} else 0
                is_closed = 1 if present_pos >= {pos_close_th} else 0

                _set_bit({tool_do_base}, 1 if moving != 0 else 0)
                _set_bit({tool_do_base + 1}, 1 if (moving_status & 0x01) != 0 else 0)
                _set_bit({tool_do_base + 2}, is_open)
                _set_bit({tool_do_base + 3}, is_closed)
                _set_bit({tool_do_base + 4}, 1)
                _set_bit({tool_do_base + 5}, 1 if present_pos != 0 else 0)
            else:
                _set_bit({tool_do_base}, 0)
                _set_bit({tool_do_base + 1}, 0)
                _set_bit({tool_do_base + 2}, 0)
                _set_bit({tool_do_base + 3}, 0)
                _set_bit({tool_do_base + 4}, 0)
                _set_bit({tool_do_base + 5}, 0)
        """)
        report_script = f"{DRL_BASE_CODE}\n{task_code}"
        success = self._send_drl_script(report_script)
        if not success:
            self.node.get_logger().error("Failed to execute RH-P12-RN(A) status report script.")
        return success

    def set_tool_do_pattern(self, tool_do_base: int, pattern: int = 0b101010) -> bool:
        bits = [(pattern >> i) & 0x01 for i in range(6)]
        task_code = textwrap.dedent(f"""
            set_tool_digital_output({tool_do_base}, {bits[0]})
            set_tool_digital_output({tool_do_base + 1}, {bits[1]})
            set_tool_digital_output({tool_do_base + 2}, {bits[2]})
            set_tool_digital_output({tool_do_base + 3}, {bits[3]})
            set_tool_digital_output({tool_do_base + 4}, {bits[4]})
            set_tool_digital_output({tool_do_base + 5}, {bits[5]})
        """)
        pattern_script = f"{DRL_BASE_CODE}\n{task_code}"
        success = self._send_drl_script(pattern_script)
        if not success:
            self.node.get_logger().error("Failed to execute tool DO pattern script.")
        return success

    def report_rhp12rna_status_to_ctrl_do(
        self,
        ctrl_do_base: int,
        pos_open_th: int = 50,
        pos_close_th: int = 690,
        slave_id: int = 1,
        start_addr: int = 285,
        reg_count: int = 7,
    ) -> bool:
        if reg_count < 7:
            reg_count = 7
        read_cmd = (
            f"\\x{slave_id:02X}"
            f"\\x03"
            f"\\x{(start_addr >> 8) & 0xFF:02X}\\x{start_addr & 0xFF:02X}"
            f"\\x{(reg_count >> 8) & 0xFF:02X}\\x{reg_count & 0xFF:02X}"
        )
        task_code = textwrap.dedent(f"""
            def _set_bit(idx, val):
                if val not in (0, 1):
                    val = 0
                set_digital_output(idx, val)

            def _get_reg(data, idx):
                base = 3 + (idx * 2)
                if data is None:
                    return 0
                try:
                    if len(data) < base + 2:
                        return 0
                    return (data[base] << 8) | data[base + 1]
                except Exception:
                    return 0

            flange_serial_write(modbus_send_make(b"{read_cmd}"))
            wait(0.05)
            res, data = flange_serial_read(0.5)
            if res > 0 and data is not None:
                byte_count = 0
                try:
                    if len(data) >= 3:
                        byte_count = data[2]
                except Exception:
                    byte_count = 0
                total_regs = byte_count // 2 if byte_count >= 2 else 0
                data_ok = 1
                reg0 = _get_reg(data, 0)
                moving = reg0 & 0xFF
                moving_status = (reg0 >> 8) & 0xFF
                is_open = 0
                is_closed = 0
                if data_ok == 1 and total_regs >= 7:
                    pos_lo = _get_reg(data, 5)
                    pos_hi = _get_reg(data, 6)
                    present_pos = (pos_hi << 16) | pos_lo
                    is_open = 1 if present_pos <= {pos_open_th} else 0
                    is_closed = 1 if present_pos >= {pos_close_th} else 0

                _set_bit({ctrl_do_base}, 1 if moving != 0 else 0)
                _set_bit({ctrl_do_base + 1}, 1 if (moving_status & 0x01) != 0 else 0)
                _set_bit({ctrl_do_base + 2}, is_open)
                _set_bit({ctrl_do_base + 3}, is_closed)
                _set_bit({ctrl_do_base + 4}, 1)
                _set_bit({ctrl_do_base + 5}, data_ok)
            else:
                _set_bit({ctrl_do_base}, 0)
                _set_bit({ctrl_do_base + 1}, 0)
                _set_bit({ctrl_do_base + 2}, 0)
                _set_bit({ctrl_do_base + 3}, 0)
                _set_bit({ctrl_do_base + 4}, 0)
                _set_bit({ctrl_do_base + 5}, 0)
        """)
        report_script = f"{DRL_BASE_CODE}\n{task_code}"
        success = self._send_drl_script(report_script)
        if not success:
            self.node.get_logger().error("Failed to execute RH-P12-RN(A) status report script (ctrl DO).")
        return success

    def set_ctrl_do_pattern_via_drl(self, ctrl_do_base: int, pattern: int = 0b101010) -> bool:
        bits = [(pattern >> i) & 0x01 for i in range(6)]
        task_code = textwrap.dedent(f"""
            set_digital_output({ctrl_do_base}, {bits[0]})
            set_digital_output({ctrl_do_base + 1}, {bits[1]})
            set_digital_output({ctrl_do_base + 2}, {bits[2]})
            set_digital_output({ctrl_do_base + 3}, {bits[3]})
            set_digital_output({ctrl_do_base + 4}, {bits[4]})
            set_digital_output({ctrl_do_base + 5}, {bits[5]})
        """)
        pattern_script = f"{DRL_BASE_CODE}\n{task_code}"
        success = self._send_drl_script(pattern_script)
        if not success:
            self.node.get_logger().error("Failed to execute ctrl DO pattern script (DRL).")
        return success

    def report_modbus_read_len_to_ctrl_do(
        self,
        ctrl_do_base: int,
        slave_id: int,
        start_addr: int,
        reg_count: int,
    ) -> bool:
        read_cmd = (
            f"\\x{slave_id:02X}"
            f"\\x03"
            f"\\x{(start_addr >> 8) & 0xFF:02X}\\x{start_addr & 0xFF:02X}"
            f"\\x{(reg_count >> 8) & 0xFF:02X}\\x{reg_count & 0xFF:02X}"
        )
        task_code = textwrap.dedent(f"""
            def _set_bit(idx, val):
                if val not in (0, 1):
                    val = 0
                set_digital_output(idx, val)

            flange_serial_write(modbus_send_make(b"{read_cmd}"))
            wait(0.05)
            res, data = flange_serial_read(0.5)
            length = 0
            if res is not None and res > 0:
                length = res
            for i in range(6):
                _set_bit({ctrl_do_base} + i, (length >> i) & 0x01)
        """)
        script = f"{DRL_BASE_CODE}\n{task_code}"
        success = self._send_drl_script(script)
        if not success:
            self.node.get_logger().error("Failed to execute modbus read length script.")
        return success

    def report_modbus_byte_count_to_ctrl_do(
        self,
        ctrl_do_base: int,
        slave_id: int,
        start_addr: int,
        reg_count: int,
    ) -> bool:
        read_cmd = (
            f"\\x{slave_id:02X}"
            f"\\x03"
            f"\\x{(start_addr >> 8) & 0xFF:02X}\\x{start_addr & 0xFF:02X}"
            f"\\x{(reg_count >> 8) & 0xFF:02X}\\x{reg_count & 0xFF:02X}"
        )
        task_code = textwrap.dedent(f"""
            def _set_bit(idx, val):
                if val not in (0, 1):
                    val = 0
                set_digital_output(idx, val)

            flange_serial_write(modbus_send_make(b"{read_cmd}"))
            wait(0.05)
            res, data = flange_serial_read(0.5)
            byte_count = 0
            if data is not None:
                try:
                    if len(data) >= 3:
                        byte_count = data[2]
                except Exception:
                    byte_count = 0
            for i in range(6):
                _set_bit({ctrl_do_base} + i, (byte_count >> i) & 0x01)
        """)
        script = f"{DRL_BASE_CODE}\n{task_code}"
        success = self._send_drl_script(script)
        if not success:
            self.node.get_logger().error("Failed to execute modbus byte count script.")
        return success

    def report_modbus_byte_to_ctrl_do(
        self,
        ctrl_do_base: int,
        slave_id: int,
        start_addr: int,
        reg_count: int,
        byte_index: int,
        shift: int = 0,
    ) -> bool:
        if byte_index < 0:
            byte_index = 0
        if shift < 0:
            shift = 0
        read_cmd = (
            f"\\x{slave_id:02X}"
            f"\\x03"
            f"\\x{(start_addr >> 8) & 0xFF:02X}\\x{start_addr & 0xFF:02X}"
            f"\\x{(reg_count >> 8) & 0xFF:02X}\\x{reg_count & 0xFF:02X}"
        )
        task_code = textwrap.dedent(f"""
            def _set_bit(idx, val):
                if val not in (0, 1):
                    val = 0
                set_digital_output(idx, val)

            flange_serial_write(modbus_send_make(b"{read_cmd}"))
            wait(0.05)
            res, data = flange_serial_read(0.5)
            valid = 0
            value = 0
            if data is not None:
                try:
                    if len(data) > {byte_index}:
                        value = data[{byte_index}]
                        valid = 1
                except Exception:
                    value = 0
                    valid = 0
            value = (value >> {shift}) & 0x3F
            for i in range(6):
                _set_bit({ctrl_do_base} + i, (value >> i) & 0x01)
            _set_bit({ctrl_do_base + 5}, valid)
        """)
        script = f"{DRL_BASE_CODE}\n{task_code}"
        success = self._send_drl_script(script)
        if not success:
            self.node.get_logger().error("Failed to execute modbus byte dump script.")
        return success

    def wait_for_grasp_completed(
        self,
        timeout_sec: float = 5.0,
        poll_sec: float = 0.1,
        slave_id: int = 9,
        start_addr: int = 0x07D0,
        reg_count: int = 3,
    ) -> bool:
        if poll_sec <= 0.0:
            poll_sec = 0.1
        max_polls = max(1, int(timeout_sec / poll_sec))
        read_cmd = (
            f"\\x{slave_id:02X}"
            f"\\x03"
            f"\\x{(start_addr >> 8) & 0xFF:02X}\\x{start_addr & 0xFF:02X}"
            f"\\x{(reg_count >> 8) & 0xFF:02X}\\x{reg_count & 0xFF:02X}"
        )

        task_code = textwrap.dedent(f"""
            def _is_grasp_done(data):
                if data is None:
                    return False
                try:
                    if len(data) < 7:
                        return False
                except Exception:
                    return False
                status = data[3]
                g_sta = (status >> 4) & 0x03
                g_obj = (status >> 6) & 0x03
                return (g_sta == 3) and (g_obj != 0)

            done = False
            for _ in range({max_polls}):
                flange_serial_write(modbus_send_make(b"{read_cmd}"))
                res, data = flange_serial_read()
                if res > 0:
                    if _is_grasp_done(data):
                        done = True
                        break
                wait({poll_sec})

            if done:
                tp_log("gripper grasp completed")
            else:
                tp_log("gripper grasp timeout")
        """)
        wait_script = f"{DRL_BASE_CODE}\n{task_code}"
        success = self._send_drl_script(wait_script)
        if not success:
            self.node.get_logger().error("Failed to execute grasp wait script.")
        return success

    def terminate(self) -> bool:
            self.node.get_logger().info("Terminating gripper connection...")
            terminate_script = "flange_serial_close()"
            success = self._send_drl_script(terminate_script)
            if success:
                self.node.get_logger().info("Gripper connection terminated successfully.")
            else:
                self.node.get_logger().error("Failed to terminate gripper connection.")
            return success

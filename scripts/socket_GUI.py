import socket
import threading
import tkinter as tk
from tkinter import messagebox, Label
import signal
import sys

class ClientGUI:
    def __init__(self, master):
        self.master = master
        master.title("UR10e 컨트롤러")

        self.client_socket = None
        self.is_connected = False
        self.detected_objects = 0

        # 상태 표시 레이블
        self.status_label = tk.Label(master, text="서버에 연결 중...", fg="orange")
        self.status_label.pack(pady=5)

        # 감지된 물체 수 표시 레이블
        self.objects_label = tk.Label(master, text="감지된 물체: 0", fg="blue", font=("Helvetica", 12, "bold"))
        self.objects_label.pack(pady=5)

        self.buttons = []

        # 카메라 프레임
        camera_frame = tk.Frame(master)
        camera_frame.pack(padx=10, pady=5)

        cam_btn = tk.Button(camera_frame, text="Cam", width=10, command=lambda: self.send_command("Cam"))
        cam_btn.pack(side=tk.LEFT, padx=5, pady=5)
        self.buttons.append(cam_btn)

        # 그립 프레임
        grip_frame = tk.Frame(master)
        grip_frame.pack(padx=10, pady=5)
        grip_frame_label = tk.Label(grip_frame, text="Pick Objects:")
        grip_frame_label.pack(side=tk.LEFT, padx=5, pady=5)

        grip_commands = ["Grip1", "Grip2", "Grip3"]
        
        for cmd in grip_commands:
            btn = tk.Button(grip_frame, text=cmd, width=8, command=lambda c=cmd: self.send_command(c))
            btn.pack(side=tk.LEFT, padx=5, pady=5)
            self.buttons.append(btn)

        # 새로운 Move 프레임
        move_frame = tk.Frame(master)
        move_frame.pack(padx=10, pady=5)
        move_frame_label = tk.Label(move_frame, text="Move To:")
        move_frame_label.pack(side=tk.LEFT, padx=5, pady=5)

        move_commands = ["Move1", "Move2", "Move3"]
        
        for cmd in move_commands:
            btn = tk.Button(move_frame, text=cmd, width=8, command=lambda c=cmd: self.send_command(c))
            btn.pack(side=tk.LEFT, padx=5, pady=5)
            self.buttons.append(btn)

        # 수정된 Load/Unload 프레임
        load_frame = tk.Frame(master)
        load_frame.pack(padx=10, pady=5)
        load_frame_label = tk.Label(load_frame, text="Machine:")
        load_frame_label.pack(side=tk.LEFT, padx=5, pady=5)

        load_btn = tk.Button(load_frame, text="Load", width=8, command=lambda: self.send_command("Load"))
        load_btn.pack(side=tk.LEFT, padx=5, pady=5)
        self.buttons.append(load_btn)

        unload_btn = tk.Button(load_frame, text="Unload", width=8, command=lambda: self.send_command("Unload"))
        unload_btn.pack(side=tk.LEFT, padx=5, pady=5)
        self.buttons.append(unload_btn)

        # 그리퍼 제어 프레임
        gripper_frame = tk.Frame(master)
        gripper_frame.pack(padx=10, pady=5)
        gripper_frame_label = tk.Label(gripper_frame, text="Gripper:")
        gripper_frame_label.pack(side=tk.LEFT, padx=5, pady=5)

        open_btn = tk.Button(gripper_frame, text="Open", width=8, command=lambda: self.send_command("Open"))
        open_btn.pack(side=tk.LEFT, padx=5, pady=5)
        self.buttons.append(open_btn)

        close_btn = tk.Button(gripper_frame, text="Close", width=8, command=lambda: self.send_command("Close"))
        close_btn.pack(side=tk.LEFT, padx=5, pady=5)
        self.buttons.append(close_btn)

        # 서버 연결 시도
        self.connect_to_server()

    def signal_handler(self, sig, frame):
        print("Ctrl+C pressed. Closing the application.")
        self.on_closing()

    def connect_to_server(self):
        try:
            self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.client_socket.connect(("127.0.0.1", 5000))
            self.is_connected = True
            self.status_label.config(text="서버에 연결됨", fg="green")
            self.enable_buttons()
            threading.Thread(target=self.receive_messages, daemon=True).start()
        except Exception as e:
            self.status_label.config(text=f"연결 시도 중..{str(e)}", fg="red")
            self.disable_buttons()
            self.master.after(5000, self.connect_to_server)  # 5초 후 재시도

    def send_command(self, command):
        if self.is_connected:
            try:
                self.client_socket.send(command.encode('utf-8'))
                print(f"서버로 명령 전송: {command}")
                self.disable_buttons()
            except Exception as e:
                messagebox.showerror("전송 오류", str(e))
                self.reset_connection()
        else:
            messagebox.showwarning("연결 없음", "서버에 연결되어 있지 않습니다. 재연결을 시도합니다.")
            self.connect_to_server()

    def receive_messages(self):
        while self.is_connected:
            try:
                response = self.client_socket.recv(1024).decode('utf-8')
                if not response:
                    break
                print(f"서버로부터 응답 수신: {response}")
                
                # 숫자만 포함된 응답인 경우 감지된 물체 수로 처리
                if response.isdigit():
                    self.detected_objects = int(response)
                    self.master.after(0, lambda: self.objects_label.config(text=f"감지된 물체: {self.detected_objects}"))
                
                if response == "complete":
                    self.master.after(0, self.enable_buttons)

                if len(response) > 8:
                    self.detected_objects = int(response[0])
                    self.master.after(0, lambda: self.objects_label.config(text=f"감지된 물체: {self.detected_objects}"))
                    self.master.after(0, self.enable_buttons)
                    
            except Exception as e:
                print(f"수신 오류: {str(e)}")
                break
        self.reset_connection()

    def enable_buttons(self):
        for btn in self.buttons:
            btn['state'] = 'normal'

    def disable_buttons(self):
        for btn in self.buttons:
            btn['state'] = 'disabled'

    def reset_connection(self):
        self.is_connected = False
        self.status_label.config(text="서버 연결 끊김. 재연결 중...", fg="orange")
        self.disable_buttons()
        self.connect_to_server()

    def on_closing(self):
        if self.is_connected:
            try:
                self.client_socket.close()
            except:
                pass
        self.master.destroy()

def main():
    root = tk.Tk()
    client_gui = ClientGUI(root)
    root.protocol("WM_DELETE_WINDOW", client_gui.on_closing)

    # SIGINT (Ctrl+C) 핸들러 설정
    signal.signal(signal.SIGINT, client_gui.signal_handler)

    # 주기적으로 이벤트 처리를 위한 함수
    def check_for_signals():
        root.after(100, check_for_signals)

    check_for_signals()
    root.mainloop()

if __name__ == "__main__":
    main()
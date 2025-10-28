import socket
import numpy as np
import pygame
import threading
import queue
import time

TRACKING_WHEEL_CIRCUMFERENCE = 2 * np.pi  # inches
DRIVE_WHEEL_CIRCUMFERENCE = 4 * np.pi  # inches
TRACKING_RADIUS = 5.0  # inches
tl = 7.25  # inches
tr = 7.25  # inches
tb = 7.75  # inches

MAX_RPM = 450


class Robot:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.left_motor_speed = 0.0
        self.right_motor_speed = 0.0
        self.left_motor_spin = "off"
        self.right_motor_spin = "off"
        self.leftSensorRotations = 0.0
        self.rightSensorRotations = 0.0
        self.backSensorRotations = 0.0

    def step(self, dt):
        sl = (
            self.left_motor_speed
            if self.left_motor_spin == "forward"
            else -self.left_motor_speed if self.left_motor_spin == "reverse" else 0.0
        )
        sr = (
            self.right_motor_speed
            if self.right_motor_spin == "forward"
            else -self.right_motor_speed if self.right_motor_spin == "reverse" else 0.0
        )

        dl = MAX_RPM / 60 * DRIVE_WHEEL_CIRCUMFERENCE * dt * sl / 100
        dr = MAX_RPM / 60 * DRIVE_WHEEL_CIRCUMFERENCE * dt * sr / 100
        self.leftSensorRotations += dl / TRACKING_WHEEL_CIRCUMFERENCE * 360
        self.rightSensorRotations += dr / TRACKING_WHEEL_CIRCUMFERENCE * 360

        d_theta = (dr - dl) / (2 * TRACKING_RADIUS)
        self.backSensorRotations += (d_theta * tb) / TRACKING_WHEEL_CIRCUMFERENCE * 360

        ds = (dl + dr) / 2

        theta_avg = self.theta + (d_theta / 2)

        dx = ds * np.cos(theta_avg)
        dy = ds * np.sin(theta_avg)

        self.x += dx
        self.y += dy
        self.theta += d_theta

    def draw(self, screen):
        robot_width = 20
        robot_height = 30

        # Convert robot position (x, y) to screen coordinates (centered around middle)
        center_x = int(self.x) + 400
        center_y = -int(self.y) + 300

        # Create a rectangle representing the robot body
        robot_surface = pygame.Surface((robot_width, robot_height), pygame.SRCALPHA)
        robot_surface.fill((255, 0, 0))  # red robot

        # Rotate the robot around its center based on heading (theta in radians)
        rotated_surface = pygame.transform.rotate(robot_surface, np.degrees(self.theta))

        # Recalculate position so rotation stays centered
        rotated_rect = rotated_surface.get_rect(center=(center_x, center_y))

        # Draw robot on the screen
        screen.blit(rotated_surface, rotated_rect.topleft)

        # Optional: draw a small heading line (to show forward direction)
        heading_length = 25
        end_x = center_x + heading_length * np.cos(-self.theta)
        end_y = center_y + heading_length * np.sin(-self.theta)
        pygame.draw.line(screen, (0, 0, 255), (center_x, center_y), (end_x, end_y), 2)


class RobotSim:
    def __init__(self, host="127.0.0.1", port=65432):
        self.host = host
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((self.host, self.port))
        self.sock.listen()
        self.robot = Robot()
        self.message_queue = queue.Queue()
        self.running = True
        self.conn = None

    def socket_thread(self):
        """Thread function for handling socket communication"""
        print(f"Socket server listening on {self.host}:{self.port}")

        while self.running:
            try:
                # Accept connection
                self.conn, addr = self.sock.accept()
                print(f"Connected by {addr}")

                while self.running:
                    try:
                        data = self.conn.recv(1024)
                        if not data:
                            break

                        msg = data.decode().strip()
                        if msg:
                            # Put message in queue for main thread to process
                            self.message_queue.put(msg)
                            print(f"Received message: {msg}")

                        # Send robot state back
                        robot_state = f"{self.robot.leftSensorRotations} | {self.robot.rightSensorRotations} | {self.robot.backSensorRotations} | {self.robot.theta}\n"
                        self.conn.sendall(robot_state.encode())

                    except socket.error as e:
                        print(f"Socket error: {e}")
                        break

            except socket.error as e:
                if self.running:
                    print(f"Socket accept error: {e}")
                    time.sleep(1)  # Wait before retrying

            finally:
                if self.conn:
                    self.conn.close()
                    self.conn = None

    def process_message_queue(self):
        """Process all messages in the queue"""
        while not self.message_queue.empty():
            try:
                msg = self.message_queue.get_nowait()
                self.process_message(msg)
            except queue.Empty:
                break

    def process_message(self, msg):
        print("Processing message:", msg)
        try:
            cmd = msg.split(" | ")[0]
            params = msg.split(" | ")[1:]

            if cmd == "set_velocity":
                motor_id = params[0]
                speed = float(params[1])
                if motor_id == "left":
                    self.robot.left_motor_speed = speed
                elif motor_id == "right":
                    self.robot.right_motor_speed = speed
                else:
                    raise ValueError(f"Unknown motor ID: {motor_id}")

            elif cmd == "stop_motor":
                motor_id = params[0]
                if motor_id == "left":
                    self.robot.left_motor_spin = "off"
                elif motor_id == "right":
                    self.robot.right_motor_spin = "off"
                else:
                    raise ValueError(f"Unknown motor ID: {motor_id}")

            elif cmd == "spin_motor":
                motor_id = params[0]
                dir = params[1]
                if dir not in ["forward", "reverse"]:
                    raise ValueError(f"Unknown spin direction: {dir}")
                if motor_id == "left":
                    self.robot.left_motor_spin = dir
                elif motor_id == "right":
                    self.robot.right_motor_spin = dir
                else:
                    raise ValueError(f"Unknown motor ID: {motor_id}")

            # Handle C++ client commands
            elif cmd == "set_turn_state":
                motor_id = params[0]
                turn_value = float(params[1])
                # Convert turn_value to direction (assuming positive = forward, negative = reverse)
                direction = "forward" if turn_value >= 0 else "reverse"
                if motor_id == "left":
                    self.robot.left_motor_spin = direction
                elif motor_id == "right":
                    self.robot.right_motor_spin = direction
                else:
                    raise ValueError(f"Unknown motor ID: {motor_id}")

            elif cmd == "set_stop":
                motor_id = params[0]
                if motor_id == "left":
                    self.robot.left_motor_spin = "off"
                    self.robot.left_motor_speed = 0.0
                elif motor_id == "right":
                    self.robot.right_motor_spin = "off"
                    self.robot.right_motor_speed = 0.0
                else:
                    raise ValueError(f"Unknown motor ID: {motor_id}")
            else:
                print(f"Unknown command: {cmd}")

        except Exception as e:
            print(f"Error processing message '{msg}': {e}")

    def start_socket_thread(self):
        """Start the socket communication thread"""
        self.socket_thread_obj = threading.Thread(
            target=self.socket_thread, daemon=True
        )
        self.socket_thread_obj.start()

    def stop(self):
        """Clean shutdown of the simulator"""
        self.running = False
        if self.conn:
            self.conn.close()
        self.sock.close()


if __name__ == "__main__":
    pygame.init()
    # launch a pygame window that is 800 x 600
    screen = pygame.display.set_mode((800, 600))
    pygame.display.set_caption("VEX Robot Simulator")
    clock = pygame.time.Clock()

    # Create simulator and start socket thread
    sim = RobotSim()
    sim.start_socket_thread()

    print("Starting pygame main loop...")
    print("Socket thread is running in background...")

    try:
        # Main pygame loop runs in main thread
        while sim.running:
            # Handle pygame events
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    sim.running = False
                    break

            # Process any incoming messages from socket thread
            sim.process_message_queue()

            # Update robot physics
            sim.robot.step(1 / 60)

            # Render
            screen.fill((255, 255, 255))
            sim.robot.draw(screen)

            # Add some status text
            font = pygame.font.Font(None, 36)
            status_text = f"Robot: x={sim.robot.x:.1f}, y={sim.robot.y:.1f}, θ={np.degrees(sim.robot.theta):.1f}°"
            text_surface = font.render(status_text, True, (0, 0, 0))
            screen.blit(text_surface, (10, 10))

            motor_text = f"Motors: L={sim.robot.left_motor_speed:.1f}% ({sim.robot.left_motor_spin}), R={sim.robot.right_motor_speed:.1f}% ({sim.robot.right_motor_spin})"
            motor_surface = font.render(motor_text, True, (0, 0, 0))
            screen.blit(motor_surface, (10, 50))
            # add more status text for encoder rotations
            encoder_text = f"Encoders: Left={sim.robot.leftSensorRotations:.1f}°, Right={sim.robot.rightSensorRotations:.1f}°, Back={sim.robot.backSensorRotations:.1f}°"
            encoder_surface = font.render(encoder_text, True, (0, 0, 0))
            screen.blit(encoder_surface, (10, 90))

            pygame.display.flip()
            clock.tick(60)

    except KeyboardInterrupt:
        print("Interrupted by user")
    finally:
        print("Shutting down...")
        sim.stop()
        pygame.quit()

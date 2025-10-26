import socket
import numpy as np
import pygame

LATERAL_OFFSET = 0.0  # inches
FORWARD_OFFSET = 7.25  # inches
TRACKING_WHEEL_CIRCUMFERENCE = 2 * np.pi  # inches
DRIVE_WHEEL_CIRCUMFERENCE = 4 * np.pi  # inches
TRACKING_RADIUS = 7.25  # inches
MAX_RPM = 450


class Robot:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.left_motor_speed = 50
        self.right_motor_speed = 45.15
        self.left_motor_spin = "forward"
        self.right_motor_spin = "forward"
        self.perpendicular_motor_rotations = 0.0
        self.parallel_motor_rotations = 0.0

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

        d_theta = (dr - dl) / (2 * TRACKING_RADIUS)

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
        self.sock.bind((self.host, self.port))
        self.sock.listen()
        self.robot = Robot()

    def process_message(self, msg):
        print("Processing message:", msg)
        cmd = msg.split("|")[0]
        params = msg.split("|")[1:]
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


if __name__ == "__main__":
    pygame.init()
    # launch a pygame window that is 800 x 600
    screen = pygame.display.set_mode((800, 600))
    clock = pygame.time.Clock()
    sim = RobotSim()
    # conn, addr = sim.sock.accept()
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                exit()
        # data = conn.recv(1024)
        # if not data:
        #     break
        # msg = data.decode()
        # sim.process_message(msg)
        # robot_state = "x:10,y:20,theta:1.57"
        # conn.sendall(robot_state.encode())

        screen.fill((255, 255, 255))
        sim.robot.draw(screen)
        sim.robot.step(1 / 60)
        pygame.display.flip()
        clock.tick(60)

import math
import pygame

FIELD_WIDTH = 144  # in inches
RESOLUTION = 800  # in pixels
RED = (255, 0, 0)
WHITE = (255, 255, 255)
BLUE = (0, 0, 255)


def S(x):  # scale
    return RESOLUTION / FIELD_WIDTH * x


def T(x):  # translate
    return RESOLUTION / 2 + S(x)


class PyGameVisualizer:
    def __init__(self):
        pygame.init()
        pygame.display.set_caption("VEX Robot Simulator")
        self.screen = pygame.display.set_mode((RESOLUTION, RESOLUTION))

    def draw(self, robot):
        self.screen.fill(WHITE)

        center_x = T(robot.x)
        center_y = T(robot.y)
        theta = -robot.theta  # Invert for pygame
        theta_rad = math.radians(theta)

        width = S(robot.width)
        length = S(robot.length)

        surface = pygame.Surface((width, length), pygame.SRCALPHA)
        surface.fill(RED)

        # Rotate the robot around its center based on heading (theta in radians)
        rotated_surface = pygame.transform.rotate(surface, theta)

        # Recalculate position so rotation stays centered
        rotated_rect = rotated_surface.get_rect(center=(center_x, center_y))

        # Draw robot on the screen
        self.screen.blit(rotated_surface, rotated_rect.topleft)

        heading_length = S(robot.length)
        end_x = center_x + heading_length * math.sin(-theta_rad)
        end_y = center_y - heading_length * math.cos(-theta_rad)
        pygame.draw.line(self.screen, BLUE, (center_x, center_y), (end_x, end_y), 2)

        pygame.display.flip()

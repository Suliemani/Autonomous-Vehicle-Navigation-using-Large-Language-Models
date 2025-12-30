import pygame
import math


class ChatInterface:
    def __init__(self, x, y, width, height):
        # Initialize chat interface with position and dimensions
        pygame.font.init()
        self.rect = pygame.Rect(x, y, width, height)
        self.font = pygame.font.SysFont("Times New Roman", 14)
        self.messages = []
        self.input_text = ""
        self.active = True

        # Color settings for chat elements
        self.bg_color = (232, 233, 237)
        self.border_color = (180, 180, 180)
        self.user_bubble_color = (255, 255, 255)
        self.gpt_bubble_color = (240, 240, 245)
        self.system_bubble_color = (220, 220, 230)

        # Text and layout settings
        self.text_color = (0, 0, 0)
        self.line_spacing = 4
        self.scroll_offset = 0
        self.bubble_max_width = width - 30
        self.input_box_height = 30

    def add_message(self, speaker, text):
        # Add new message to chat history
        self.messages.append((speaker, text))
        self.scroll_offset = 0

    def handle_event(self, event):
        # Process keyboard input for chat
        if event.type == pygame.KEYDOWN and self.active:
            if event.key == pygame.K_RETURN:
                line = self.input_text.strip()
                self.input_text = ""
                return line
            elif event.key == pygame.K_BACKSPACE:
                self.input_text = self.input_text[:-1]
            else:
                if event.unicode.isprintable():
                    self.input_text += event.unicode
        return None

    def scroll(self, amount):
        # Handle chat history scrolling
        self.scroll_offset += amount
        max_lines = len(self.messages)
        if self.scroll_offset < 0:
            self.scroll_offset = 0
        if max_lines > 0 and self.scroll_offset > max_lines - 1:
            self.scroll_offset = max_lines - 1

    def render(self, screen):
        # Draw chat interface on screen
        pygame.draw.rect(screen, self.bg_color, self.rect)
        pygame.draw.rect(screen, self.border_color, self.rect, 2)

        # Calculate areas for message history and input box
        margin = 10
        inner_rect = self.rect.inflate(-2 * margin, -2 * margin)
        history_height = inner_rect.height - self.input_box_height - margin
        history_rect = pygame.Rect(
            inner_rect.x, inner_rect.y, inner_rect.width, history_height
        )
        input_rect = pygame.Rect(
            inner_rect.x,
            history_rect.bottom + margin,
            inner_rect.width,
            self.input_box_height,
        )

        # Save current clipping area
        clip = screen.get_clip()
        screen.set_clip(history_rect)

        # Render message history from bottom up
        current_y = history_rect.bottom
        left_x = history_rect.x + 10
        right_x = history_rect.right - 10

        for i in range(len(self.messages) - 1, -1, -1):
            idx = i - self.scroll_offset
            if idx < 0:
                continue

            speaker, text = self.messages[idx]
            label_surf = self.font.render(speaker, True, (50, 50, 50))
            label_h = label_surf.get_height()

            # Wrap text to fit in chat bubbles
            lines = self._wrap_text(text, self.font, self.bubble_max_width)
            line_surfs = [self.font.render(ln, True, (0, 0, 0)) for ln in lines]
            total_text_h = (
                sum(s.get_height() for s in line_surfs)
                + self.line_spacing * (len(lines) - 1)
            )

            # Calculate bubble dimensions
            bubble_pad = 8
            bubble_w = (
                max(
                    max((s.get_width() for s in line_surfs), default=0),
                    label_surf.get_width(),
                )
                + bubble_pad * 2
            )
            bubble_h = label_h + self.line_spacing + total_text_h + bubble_pad * 2

            # Position bubble and skip if outside visible area
            current_y -= (bubble_h + 8)
            if current_y > history_rect.bottom:
                break
            if current_y + bubble_h < history_rect.y:
                continue

            # Position bubble based on speaker
            if speaker.lower() == "you":
                bubble_x = right_x - bubble_w
                bubble_color = self.user_bubble_color
            elif speaker.lower() == "vehicle ai":
                bubble_x = left_x - 15
                bubble_color = self.gpt_bubble_color
            else:
                bubble_x = left_x - 15
                bubble_color = self.system_bubble_color

            # Draw chat bubble
            bubble_rect = pygame.Rect(bubble_x, current_y, bubble_w, bubble_h)
            pygame.draw.rect(screen, bubble_color, bubble_rect, border_radius=6)

            # Render text inside bubble
            line_y = current_y + bubble_pad
            screen.blit(label_surf, (bubble_x + bubble_pad, line_y))
            line_y += label_h + self.line_spacing

            for srf in line_surfs:
                screen.blit(srf, (bubble_x + bubble_pad, line_y))
                line_y += srf.get_height() + self.line_spacing

        # Restore clipping area
        screen.set_clip(clip)

        # Draw input box
        pygame.draw.rect(screen, (220, 220, 230), input_rect, border_radius=5)
        pygame.draw.rect(screen, self.border_color, input_rect, 2, border_radius=5)
        typed_surf = self.font.render(self.input_text, True, (0, 0, 0))
        screen.blit(typed_surf, (input_rect.x + 10, input_rect.y + 5))

    def _wrap_text(self, text, font, max_width):
        # Helper to wrap long text into multiple lines
        words = text.split()
        lines = []
        current_line = ""

        for w in words:
            test_line = (current_line + " " + w).strip()
            w_width, _ = font.size(test_line)
            if w_width <= max_width:
                current_line = test_line
            else:
                if current_line:
                    lines.append(current_line)
                current_line = w

        if current_line:
            lines.append(current_line)

        return lines


def point_line_dist(px, py, x1, y1, x2, y2):
    # Calculate shortest distance between point and line segment
    line_len_sq = (x2 - x1) ** 2 + (y2 - y1) ** 2
    if line_len_sq < 1e-9:
        return math.hypot(px - x1, py - y1)

    # Find projection point on line
    t = ((px - x1) * (x2 - x1) + (py - y1) * (y2 - y1)) / line_len_sq
    t = max(0, min(1, t))
    projx = x1 + t * (x2 - x1)
    projy = y1 + t * (y2 - y1)

    # Return distance to projection point
    return math.hypot(px - projx, py - projy)


def detect_route_hover(mouse_pos, route_segments, threshold=6):
    # Check if mouse is hovering over any route segment
    mx, my = mouse_pos
    for ((sx1, sy1), (sx2, sy2)) in route_segments:
        dist = point_line_dist(mx, my, sx1, sy1, sx2, sy2)
        if dist < threshold:
            return True
    return False

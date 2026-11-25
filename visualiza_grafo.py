"""Abre uma janela Pygame para inspecionar o grafo de lib.grafo."""

from collections import Counter
import math

import pygame

from lib.caminhos import Cruz, Hori, Nada, Safe, Vert, mapa
from lib.grafo import Noh, grafo


BACKGROUND = (248, 249, 251)
GRID_LINE = (194, 200, 208)
STRAIGHT_COLOR = (40, 120, 181)
TURN_COLOR = (224, 122, 45)
TEXT_COLOR = (45, 50, 58)
MARGIN = 32
DRAW_SIDE_MARGIN = 50
TOP = 52
FOOTER = 76

# Losango: L fica acima de O; N e S ficam nas laterais.
DIRECTION_OFFSETS = {
    Noh.NORTE: (-1, 0),
    Noh.SUL: (1, 0),
    Noh.LESTE: (0, -1),
    Noh.OESTE: (0, 1),
}
DIRECTION_COLORS = {
    Noh.NORTE: (209, 73, 91),
    Noh.SUL: (39, 125, 161),
    Noh.LESTE: (42, 157, 143),
    Noh.OESTE: (128, 93, 147),
}


def cell_color(cell):
    if isinstance(cell, Safe):
        return (217, 240, 226)
    if isinstance(cell, Cruz):
        return (232, 237, 245)
    if isinstance(cell, (Vert, Hori)):
        return (247, 247, 247)
    if isinstance(cell, Nada):
        return (236, 236, 236)
    return (255, 255, 255)


def find_duplicate_edges(graph):
    duplicates = []
    for node in graph.nós.values():
        counts = Counter(neighbor.key for neighbor in node.adj)
        duplicates.extend(
            (node.key, key, count)
            for key, count in counts.items()
            if count > 1
        )
    return duplicates


def collect_edges(graph):
    counts = Counter(
        (node.key, neighbor.key)
        for node in graph.nós.values()
        for neighbor in node.adj
    )
    rendered = []
    seen_turn_pairs = set()

    for (source_key, target_key), count in counts.items():
        source = graph.nós[source_key]
        target = graph.nós[target_key]
        is_turn = source.posicao == target.posicao
        reverse_count = counts.get((target_key, source_key), 0)

        if is_turn and reverse_count:
            pair = frozenset((source_key, target_key))
            if pair in seen_turn_pairs:
                continue
            seen_turn_pairs.add(pair)

            reciprocal_count = min(count, reverse_count)
            rendered.append((source, target, True, True, reciprocal_count))
            if count > reciprocal_count:
                rendered.append((source, target, True, False, count - reciprocal_count))
            if reverse_count > reciprocal_count:
                rendered.append((target, source, True, False, reverse_count - reciprocal_count))
        else:
            rendered.append((source, target, is_turn, False, count))

    return rendered


def node_position(node, cell_width, cell_height, origin):
    row, col = node.posicao
    dx, dy = DIRECTION_OFFSETS[node.direcao]
    center_x = origin[0] + (col + 0.5) * cell_width
    center_y = origin[1] + (row + 0.5) * cell_height
    offset_x = cell_width * 0.34
    offset_y = cell_height * 0.34
    return round(center_x + dx * offset_x), round(center_y + dy * offset_y)


def draw_arrow(surface, position, direction, size, half_width, color):
    dx, dy = direction
    length = math.hypot(dx, dy)
    if length == 0:
        return
    ux, uy = dx / length, dy / length
    px, py = -uy, ux
    x, y = position
    tip = (round(x + ux * size), round(y + uy * size))
    base = (x - ux * size * 0.8, y - uy * size * 0.8)
    points = [
        tip,
        (round(base[0] + px * half_width), round(base[1] + py * half_width)),
        (round(base[0] - px * half_width), round(base[1] - py * half_width)),
    ]
    pygame.draw.polygon(surface, color, points)
    pygame.draw.polygon(surface, (255, 255, 255), points, 1)


def draw_dashed_line(surface, color, start, end, width, scale):
    dx, dy = end[0] - start[0], end[1] - start[1]
    length = math.hypot(dx, dy)
    if length == 0:
        return
    ux, uy = dx / length, dy / length
    dash = max(4, scale * 0.10)
    gap = max(3, scale * 0.07)
    offset = 0
    while offset < length:
        finish = min(offset + dash, length)
        p1 = (round(start[0] + ux * offset), round(start[1] + uy * offset))
        p2 = (round(start[0] + ux * finish), round(start[1] + uy * finish))
        pygame.draw.line(surface, color, p1, p2, width)
        offset += dash + gap


def distance_to_segment(point, start, end):
    dx, dy = end[0] - start[0], end[1] - start[1]
    length_squared = dx * dx + dy * dy
    if length_squared == 0:
        return math.dist(point, start)
    t = ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy) / length_squared
    t = max(0, min(1, t))
    nearest = (start[0] + t * dx, start[1] + t * dy)
    return math.dist(point, nearest)


def describe_edge(edge, graph):
    source, target, _, bidirectional, multiplicity = edge
    operator = "<->" if bidirectional else "->"
    description = (
        f"{source.posicao}/{source.direcao} {operator} "
        f"{target.posicao}/{target.direcao}; "
        f"peso {graph.peso(source.key, target.key)}"
    )
    if multiplicity > 1:
        description += f"; {multiplicity} paralelas"
    return description


def draw_scene(surface, graph, grid, fonts):
    width, height = surface.get_size()
    rows, cols = len(grid), len(grid[0])
    available_width = max(1, width - 2 * DRAW_SIDE_MARGIN)
    available_height = max(1, height - TOP - FOOTER)
    cell_width = available_width / cols
    cell_height = available_height / rows
    scale = min(cell_width, cell_height)
    origin = (DRAW_SIDE_MARGIN, TOP)
    node_radius = max(3, round(scale * 0.075))
    edges = collect_edges(graph)
    duplicates = find_duplicate_edges(graph)
    duplicate_count = sum(count - 1 for _, _, count in duplicates)
    directed_count = sum(len(node.adj) for node in graph.nós.values())

    surface.fill(BACKGROUND)
    header = (
        f"Grafo: {len(graph.nós)} nós | {directed_count} arcos | "
        f"{len(edges)} segmentos | {duplicate_count} duplicadas"
    )
    surface.blit(fonts["header"].render(header, True, TEXT_COLOR), (MARGIN, 14))

    for row, line in enumerate(grid):
        for col, cell in enumerate(line):
            x = round(origin[0] + col * cell_width)
            y = round(origin[1] + row * cell_height)
            right = round(origin[0] + (col + 1) * cell_width)
            bottom = round(origin[1] + (row + 1) * cell_height)
            rect = pygame.Rect(
                x,
                y,
                right - x,
                bottom - y,
            )
            pygame.draw.rect(surface, cell_color(cell), rect)
            pygame.draw.rect(surface, GRID_LINE, rect, 1)
            label = fonts["small"].render(f"{row},{col}", True, (110, 117, 127))
            surface.blit(label, (rect.x + 3, rect.y + 2))

    edge_data = []
    for edge in edges:
        source, target, is_turn, bidirectional, _ = edge
        start = node_position(source, cell_width, cell_height, origin)
        end = node_position(target, cell_width, cell_height, origin)
        color = TURN_COLOR if is_turn else STRAIGHT_COLOR
        edge_data.append((edge, start, end, color))

    mouse = pygame.mouse.get_pos()
    hovered = None
    nearest_distance = max(7, round(scale * 0.12))
    for edge, start, end, color in edge_data:
        distance = distance_to_segment(mouse, start, end)
        if distance < nearest_distance:
            nearest_distance = distance
            hovered = edge

    line_width = max(2, round(scale * 0.035))
    for edge, start, end, color in edge_data:
        is_turn = edge[2]
        width = line_width + 2 if edge is hovered else line_width
        if edge is hovered:
            pygame.draw.line(surface, (255, 255, 255), start, end, width + 4)
        if is_turn:
            draw_dashed_line(surface, color, start, end, width, scale)
        else:
            pygame.draw.line(surface, color, start, end, width)

    for node in graph.nós.values():
        center = node_position(node, cell_width, cell_height, origin)
        color = DIRECTION_COLORS[node.direcao]
        pygame.draw.circle(surface, color, center, node_radius)
        pygame.draw.circle(surface, (255, 255, 255), center, node_radius, 1)
        label = fonts["node"].render(node.direcao, True, (255, 255, 255))
        surface.blit(label, label.get_rect(center=center))

    for edge, start, end, color in edge_data:
        _, _, _, bidirectional, _ = edge
        dx, dy = end[0] - start[0], end[1] - start[1]
        length = math.hypot(dx, dy)
        if bidirectional:
            size = max(3, round(scale * 0.065))
            half_width = max(3, round(scale * 0.055))
            end_margin = node_radius + size + 1
            t = max(0.5, min(0.95, 1 - end_margin / length))
            arrow_specs = ((t, 1), (1 - t, -1))
        else:
            size = max(5, round(scale * 0.10))
            half_width = max(4, round(scale * 0.075))
            end_margin = node_radius + size + 1
            t = max(0.5, min(0.95, 1 - end_margin / length))
            arrow_specs = ((t, 1),)

        for t, direction in arrow_specs:
            position = (round(start[0] + dx * t), round(start[1] + dy * t))
            draw_arrow(
                surface,
                position,
                (dx * direction, dy * direction),
                size,
                half_width,
                color,
            )

    legend_y = height - 52
    pygame.draw.line(surface, STRAIGHT_COLOR, (MARGIN, legend_y), (MARGIN + 42, legend_y), 3)
    draw_arrow(surface, (MARGIN + 35, legend_y), (1, 0), 6, 4, STRAIGHT_COLOR)
    surface.blit(
        fonts["small"].render("entre posições", True, TEXT_COLOR),
        (MARGIN + 50, legend_y - 8),
    )

    turn_x = MARGIN + 210
    draw_dashed_line(surface, TURN_COLOR, (turn_x, legend_y), (turn_x + 42, legend_y), 3, scale)
    draw_arrow(surface, (turn_x + 12, legend_y), (-1, 0), 4, 3, TURN_COLOR)
    draw_arrow(surface, (turn_x + 34, legend_y), (1, 0), 4, 3, TURN_COLOR)
    surface.blit(
        fonts["small"].render("curva recíproca", True, TEXT_COLOR),
        (turn_x + 50, legend_y - 8),
    )

    if hovered:
        info = describe_edge(hovered, graph)
    else:
        info = "L acima de O; N/S nas laterais. Passe o mouse sobre uma aresta para inspecioná-la."
    surface.blit(fonts["small"].render(info, True, TEXT_COLOR), (MARGIN, height - 25))

    pygame.display.flip()


def main():
    pygame.init()
    pygame.display.set_caption("Visualiza grafo")
    window_size = (
        max(900, len(mapa[0]) * 58 + MARGIN * 2),
        max(680, len(mapa) * 58 + TOP + FOOTER),
    )
    surface = pygame.display.set_mode(window_size, pygame.RESIZABLE)
    fonts = {
        "header": pygame.font.SysFont("monospace", 18, bold=True),
        "small": pygame.font.SysFont("monospace", 12),
        "node": pygame.font.SysFont("monospace", 11, bold=True),
    }
    clock = pygame.time.Clock()
    running = True

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                running = False
            elif event.type == pygame.VIDEORESIZE:
                surface = pygame.display.set_mode(event.size, pygame.RESIZABLE)

        draw_scene(surface, grafo, mapa, fonts)
        clock.tick(30)

    pygame.quit()


if __name__ == "__main__":
    main()

import customtkinter as ctk
import math


class ServoVisualization(ctk.CTkCanvas):
    """Widget for servo motor visualization with angle indicators."""

    def __init__(self, master, **kwargs):
        super().__init__(master, bg="#1e1e1e", highlightthickness=0, **kwargs)

        self.servo_angles = [90.0, 90.0, 90.0]  # 0–180°
        self.servo_colors = ["#FF6B6B", "#4ECDC4", "#FFE66D"]
        self.servo_labels = ["ROLL", "PITCH", "YAW"]

        # IDs des éléments graphiques par servo
        self.servo_items = [dict() for _ in range(3)]
        # géométrie courante (centres, rayons, etc.)
        self.servo_layout = [dict() for _ in range(3)]

        self.bind("<Configure>", self._on_resize)

    def _on_resize(self, event):
        self._layout_servos()
        self._update_servo_graphics()

    def _layout_servos(self):
        """Calcule la géométrie en fonction de la taille du canvas."""
        width = self.winfo_width()
        height = self.winfo_height()

        if width < 50 or height < 50:
            return

        servo_width = width // 3
        bar_length = min(servo_width * 0.5, height * 0.35)
        bar_width = 12

        for i in range(3):
            center_x = servo_width * i + servo_width // 2
            center_y = height // 2 + 10
            bg_radius = bar_length * 0.55

            layout = self.servo_layout[i]
            layout["center_x"] = center_x
            layout["center_y"] = center_y
            layout["bg_radius"] = bg_radius
            layout["bar_length"] = bar_length
            layout["bar_width"] = bar_width

            items = self.servo_items[i]

            # Création initiale
            if not items:
                # fond
                items["bg"] = self.create_oval(
                    center_x - bg_radius, center_y - bg_radius,
                    center_x + bg_radius, center_y + bg_radius,
                    outline=self.servo_colors[i], width=2, fill="#2b2b2b"
                )

                # arc
                items["arc"] = self.create_arc(
                    center_x - bg_radius + 10, center_y - bg_radius + 10,
                    center_x + bg_radius - 10, center_y + bg_radius - 10,
                    start=90, extent=0,
                    outline=self.servo_colors[i], width=3, style="arc"
                )

                # barre
                items["bar"] = self.create_line(
                    center_x, center_y, center_x, center_y,
                    fill=self.servo_colors[i], width=bar_width, capstyle="round"
                )

                # axe
                axis_radius = 8
                items["axis"] = self.create_oval(
                    center_x - axis_radius, center_y - axis_radius,
                    center_x + axis_radius, center_y + axis_radius,
                    fill="#1e1e1e", outline=self.servo_colors[i], width=3
                )

                # caps
                cap_radius = bar_width // 2 + 2
                items["cap1"] = self.create_oval(
                    center_x - cap_radius, center_y - cap_radius,
                    center_x + cap_radius, center_y + cap_radius,
                    fill=self.servo_colors[i], outline="white", width=1
                )
                items["cap2"] = self.create_oval(
                    center_x - cap_radius, center_y - cap_radius,
                    center_x + cap_radius, center_y + cap_radius,
                    fill=self.servo_colors[i], outline="white", width=1
                )

                # marqueurs (0°, 90°, 180°)
                marker_radius = bg_radius + 15
                for marker_angle, marker_text in [(90, "0°"), (0, "90°"), (-90, "180°")]:
                    m_rad = math.radians(marker_angle)
                    m_x = center_x + marker_radius * math.cos(m_rad)
                    m_y = center_y - marker_radius * math.sin(m_rad)
                    self.create_text(
                        m_x, m_y, text=marker_text,
                        fill="gray", font=("Arial", 8)
                    )

                # label
                items["label"] = self.create_text(
                    center_x, center_y - bg_radius - 25,
                    text=self.servo_labels[i],
                    fill="white", font=("Arial", 12, "bold")
                )

                # texte angle
                items["angle_text"] = self.create_text(
                    center_x, center_y + bg_radius + 25,
                    text=f"{self.servo_angles[i]:.1f}°",
                    fill=self.servo_colors[i], font=("Arial", 13, "bold")
                )
            else:
                # si on a déjà les items, on réadapte juste à la nouvelle taille
                self.coords(
                    items["bg"],
                    center_x - bg_radius, center_y - bg_radius,
                    center_x + bg_radius, center_y + bg_radius,
                )
                self.coords(
                    items["axis"],
                    center_x - 8, center_y - 8,
                    center_x + 8, center_y + 8
                )
                self.coords(
                    items["label"],
                    center_x, center_y - bg_radius - 25
                )
                self.coords(
                    items["angle_text"],
                    center_x, center_y + bg_radius + 25
                )

    def _update_servo_graphics(self):
        """Met à jour bras + arc + texte d'angle pour chaque servo."""
        for i in range(3):
            layout = self.servo_layout[i]
            items = self.servo_items[i]
            if not layout or not items:
                continue

            center_x = layout["center_x"]
            center_y = layout["center_y"]
            bg_radius = layout["bg_radius"]
            bar_length = layout["bar_length"]
            bar_width = layout["bar_width"]

            angle = float(self.servo_angles[i])

            # arc
            arc_start = 90
            arc_extent = -angle
            self.coords(
                items["arc"],
                center_x - bg_radius + 10, center_y - bg_radius + 10,
                center_x + bg_radius - 10, center_y + bg_radius - 10
            )
            self.itemconfigure(items["arc"], start=arc_start, extent=arc_extent)

            # barre
            angle_rad = math.radians(angle - 90.0)  # 90° = horizontal
            half_length = bar_length / 2.0
            x1 = center_x - half_length * math.cos(angle_rad)
            y1 = center_y - half_length * math.sin(angle_rad)
            x2 = center_x + half_length * math.cos(angle_rad)
            y2 = center_y + half_length * math.sin(angle_rad)

            self.coords(items["bar"], x1, y1, x2, y2)

            # caps
            cap_radius = bar_width // 2 + 2
            self.coords(
                items["cap1"],
                x1 - cap_radius, y1 - cap_radius,
                x1 + cap_radius, y1 + cap_radius
            )
            self.coords(
                items["cap2"],
                x2 - cap_radius, y2 - cap_radius,
                x2 + cap_radius, y2 + cap_radius
            )

            # texte
            self.itemconfigure(
                items["angle_text"],
                text=f"{angle:.1f}°",
                fill=self.servo_colors[i]
            )

    def update_servos(self, angles):
        """
        Update servo angles.

        Args:
            angles: list of 3 angles in degrees (0-180)
        """
        self.servo_angles = [max(0.0, min(180.0, float(a))) for a in angles[:3]]
        self._update_servo_graphics()

    def set_servo_labels(self, labels):
        if len(labels) != 3:
            return
        self.servo_labels = labels
        for i in range(3):
            items = self.servo_items[i]
            if "label" in items:
                self.itemconfigure(items["label"], text=self.servo_labels[i])

    def set_servo_colors(self, colors):
        if len(colors) != 3:
            return
        self.servo_colors = colors
        for i in range(3):
            items = self.servo_items[i]
            if not items:
                continue
            self.itemconfigure(items["bg"], outline=self.servo_colors[i])
            self.itemconfigure(items["arc"], outline=self.servo_colors[i])
            self.itemconfigure(items["bar"], fill=self.servo_colors[i])
            self.itemconfigure(items["axis"], outline=self.servo_colors[i])
            self.itemconfigure(items["angle_text"], fill=self.servo_colors[i])


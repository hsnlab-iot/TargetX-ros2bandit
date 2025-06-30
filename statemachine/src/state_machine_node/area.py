from typing import List, Optional
from shapely.geometry import Polygon, Point

class Area:
    def __init__(self, area_id: str):
        self.id = area_id

    def setup(self, yaml_area):
        self.type = yaml_area.get("area_type", None)
        self.tf = yaml_area.get("tf", None)
        self.ref = yaml_area.get("ref", None)
        self.points = yaml_area.get("points", None)
        self.holes = yaml_area.get("holes", None)
        self.inverse = yaml_area.get("inverse", False)

    def inside(self, point):
        if not self.points:
            raise ValueError("Points are not defined for the area.")

        # Create the main polygon
        polygon = Polygon(self.points, holes=self.holes if self.holes else None)

        # Check if the point is inside the polygon
        point_obj = Point(point[0], point[1])
        is_inside = polygon.contains(point_obj)

        # If inverse is True, reverse the result
        return not is_inside if self.inverse else is_inside

    def __repr__(self):
        return f"Area(id={self.id}, type={self.type}, points={self.points}, inverse={self.inverse})"

class Point_input:
    def __init__(self, coords, rotation_angle, image_collection, wait_time):
        self.coords = coords
        self.angle = rotation_angle
        self.image_collection = image_collection
        self.wait_time = wait_time

    def __str__(self):
        return ("Coordinates: " + str(self.coords) + "\nAngle: " + str(self.angle) + "\nCollect Image: " + self.image_collection + "\nWait Time: " + str(self.wait_time) + "\n")

    def return_data(self):
        return [
                self.coords,
                self.wait_time,
                self.angle * 3.14 / 180,
                self.image_collection  == "Yes",
            ]
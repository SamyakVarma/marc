

class KeyboardCoordinates:
    def __init__(self):
        # Dictionary to store the coordinates of each key
        self.key_coordinates = {
            "A": (10.0, 2.0), "B": (13.4, 3.4), "C": (9.6, 3.4), "D": (8.6, 5.3), "E": (8.1, 7.2),
            "F": (10.5, 5.3), "G": (12.4, 5.3), "H": (14.3, 5.3), "I": (17.6, 7.2), "J": (16.2, 5.3),
            "K": (18.1, 5.3), "L": (20.0, 5.3), "M": (17.2, 3.4), "N": (15.2, 3.4), "O": (19.5, 7.2),
            "P": (21.4, 7.2), "Q": (4.3, 7.2), "R": (10.0, 7.2), "S": (10.0,1.0), "T": (10.0, 0.0),
            "U": (15.7, 7.2), "V": (11.5, 3.4), "W": (6.2, 7.2), "X": (7.6, 3.4), "Y": (13.8, 7.2),
            "Z": (5.7, 3.4), "backspace": (27.0, 9.0)
        }

    def get_coordinates(self, key):
        """Retrieve the coordinates of a given key."""
        key = key.upper()  # Convert key to uppercase to ensure case insensitivity
        if key in self.key_coordinates:
            return self.key_coordinates[key]
        else:
            return f"Key '{key}' not found in the layout."

# Example usage
if __name__ == "__main__":
    keyboard = KeyboardCoordinates()
    key_to_find = input("Enter the key to find its coordinates: ")
    coordinates = keyboard.get_coordinates(key_to_find)
    print(f"Coordinates for '{key_to_find}': {coordinates}")
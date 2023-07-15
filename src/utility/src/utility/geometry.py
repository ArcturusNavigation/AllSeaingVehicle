class Vec2D:
    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y

    def norm(self):
        return (self.x ** 2 + self.y ** 2) ** 0.5

    def dot(self, other):
        return self.x * other.x + self.y + other.y

    def __add__(self, other):
        if isinstance(other, (int, float)):
            return Vec2D(self.x + other, self.y + other)
        elif isinstance(other, Vec2D):
            return Vec2D(self.x + other.x, self.y + other.y)
        else:
            raise TypeError

    def __radd__(self, other):
        return self + other

    def __sub__(self, other):
        if isinstance(other, (int, float)):
            return Vec2D(self.x - other, self.y - other)
        elif isinstance(other, Vec2D):
            return Vec2D(self.x - other.x, self.y - other.y)
        else:
            raise TypeError

    def __rsub__(self, other):
        return self - other

    def __mul__(self, other):
        if isinstance(other, (int, float)):
            return Vec2D(self.x * other, self.y * other)
        else:
            raise TypeError

    def __rmul__(self, other):
        return self * other

    def __div__(self, other):
        if isinstance(other, (int, float)):
            return Vec2D(self.x / other, self.y / other)
        else:
            raise TypeError

    def __rdiv__(self, other):
        return self / other

    def __repr__(self):
        return f"Vec2D({self.x}, {self.y})"

    def __str__(self):
        return f"<{self.x}, {self.y}>"
#include "relative_localization/pose.h"

namespace ground_truth {

Coordinates::Coordinates(const int& x, const int& y) : x_(x), y_(y) {}

Coordinates::~Coordinates() {}

int Coordinates::getX() const { return x_; }

int Coordinates::getY() const { return y_; }

bool almostEqual(const float& x, const float& y) {
  const double epsilon = 1E-5;
  if (x == 0) return std::fabs(y) <= epsilon;
  if (y == 0) return std::fabs(x) <= epsilon;
  return std::fabs(x - y) / std::max(std::fabs(x), std::fabs(y)) <= epsilon;
}

float dot(const Vector3& lhs, const Vector3& rhs) {
  return lhs.getX() * rhs.getX() + lhs.getY() * rhs.getY() +
         lhs.getZ() * rhs.getZ();
}

Vector3 cross(const Vector3& lhs, const Vector3& rhs) {
  return Vector3(lhs.getY() * rhs.getZ() - rhs.getY() * lhs.getZ(),
                 lhs.getZ() * rhs.getX() - rhs.getZ() * lhs.getX(),
                 lhs.getX() * rhs.getY() - rhs.getX() * lhs.getY());
}

// Constructors and destructor
Vector3::Vector3() : x_(0.0), y_(0.0), z_(0.0) {}
Vector3::Vector3(const float& x, const float& y, const float& z)
    : x_(x), y_(y), z_(z){};
Vector3::~Vector3() {}

// Copy constructor and assignment operators
Vector3::Vector3(const Vector3& other)
    : x_(other.getX()), y_(other.getY()), z_(other.getZ()) {}

Vector3& Vector3::operator=(const Vector3& other) {
  x_ = other.getX();
  y_ = other.getY();
  z_ = other.getZ();
  return *this;
}

// Set and get data
float Vector3::getX() const { return x_; }
float Vector3::getY() const { return y_; }
float Vector3::getZ() const { return z_; }
void Vector3::setX(const float& x) { x_ = x; }
void Vector3::setY(const float& y) { y_ = y; }
void Vector3::setZ(const float& z) { z_ = z; }
void Vector3::setAbs() {
  x_ = std::fabs(x_);
  y_ = std::fabs(y_);
  z_ = std::fabs(z_);
}
float Vector3::getModule() const {
  return pow(x_ * x_ + y_ * y_ + z_ * z_, 0.5);
}

// Overload operators
bool operator==(const Vector3& lhs, const Vector3& rhs) {
  return almostEqual(lhs.getX(), rhs.getX()) &&
         almostEqual(lhs.getY(), rhs.getY()) &&
         almostEqual(lhs.getZ(), rhs.getZ());
}

bool operator!=(const Vector3& lhs, const Vector3& rhs) {
  return !(lhs == rhs);
}

Vector3 operator+(const Vector3& lhs, const Vector3& rhs) {
  return Vector3(lhs.getX() + rhs.getX(), lhs.getY() + rhs.getY(),
                 lhs.getZ() + rhs.getZ());
}

Vector3 operator-(const Vector3& lhs, const Vector3& rhs) {
  return Vector3(lhs.getX() - rhs.getX(), lhs.getY() - rhs.getY(),
                 lhs.getZ() - rhs.getZ());
}

Vector3 operator*(const float& lhs, const Vector3& rhs) {
  return Vector3(rhs.getX() * lhs, rhs.getY() * lhs, rhs.getZ() * lhs);
}

// Method for rotation of vector obtained from:
// https://gamedev.stackexchange.com/questions/28395/rotating-vector3-by-a-quaternion
Vector3 operator*(const Quaternion& lhs, const Vector3& rhs) {
  const Vector3 u(lhs.getX(), lhs.getY(), lhs.getZ());
  const Vector3 rot_1 = 2 * dot(u, rhs) * u;
  const Vector3 rot_2 = (pow(lhs.getW(), 2) - dot(u, u)) * rhs;
  const Vector3 rot_3 = 2 * lhs.getW() * cross(u, rhs);

  return rot_1 + rot_2 + rot_3;
}

Vector3 operator/(const Vector3& lhs, const float& rhs) {
  return Vector3(lhs.getX() / rhs, lhs.getY() / rhs, lhs.getZ() / rhs);
}

// Constructors and destructor
Euler::Euler() : r_(0.0), p_(0.0), y_(0.0) {}
Euler::Euler(const float& r, const float& p, const float& y)
    : r_(r), p_(p), y_(y) {}
Euler::~Euler() {}

// Set and get data
float Euler::getR() const { return r_; }
float Euler::getP() const { return p_; }
float Euler::getY() const { return y_; }
void Euler::setR(const float& r) { r_ = r; }
void Euler::setP(const float& p) { p_ = p; }
void Euler::setY(const float& y) { y_ = y; }

Vector3 Euler::toVector() const {
  return Vector3(cos(y_) * cos(p_), sin(y_) * cos(p_), sin(p_));
}

// Constructors and destructor
Quaternion::Quaternion() : x_(0.0), y_(0.0), z_(0.0), w_(0.0) {}
Quaternion::Quaternion(const float& x, const float& y, const float& z,
                       const float& w) {
  const float magnitude =
      pow((pow(x, 2) + pow(y, 2) + pow(z, 2) + pow(w, 2)), 0.5);
  x_ = x / magnitude;
  y_ = y / magnitude;
  z_ = z / magnitude;
  w_ = w / magnitude;
}

Quaternion::Quaternion(const Euler& euler) {
  double cy = cos(euler.getY() * 0.5);
  double sy = sin(euler.getY() * 0.5);
  double cp = cos(euler.getP() * 0.5);
  double sp = sin(euler.getP() * 0.5);
  double cr = cos(euler.getR() * 0.5);
  double sr = sin(euler.getR() * 0.5);

  x_ = sr * cp * cy - cr * sp * sy;
  y_ = cr * sp * cy + sr * cp * sy;
  z_ = cr * cp * sy - sr * sp * cy;
  w_ = cr * cp * cy + sr * sp * sy;
}
Quaternion::~Quaternion() {}

// Copy constructor and assignment operators
Quaternion::Quaternion(const Quaternion& other)
    : x_(other.getX()), y_(other.getY()), z_(other.getZ()), w_(other.getW()) {}

Quaternion& Quaternion::operator=(const Quaternion& other) {
  x_ = other.getX();
  y_ = other.getY();
  z_ = other.getZ();
  w_ = other.getW();
  return *this;
}

// Set and get data
float Quaternion::getX() const { return x_; }
float Quaternion::getY() const { return y_; }
float Quaternion::getZ() const { return z_; }
float Quaternion::getW() const { return w_; }
void Quaternion::setX(const float& x) { x_ = x; }
void Quaternion::setY(const float& y) { y_ = y; }
void Quaternion::setZ(const float& z) { z_ = z; }
void Quaternion::setW(const float& w) { w_ = w; }

// Overload operators
bool operator==(const Quaternion& lhs, const Quaternion& rhs) {
  return almostEqual(lhs.getX(), rhs.getX()) &&
         almostEqual(lhs.getY(), rhs.getY()) &&
         almostEqual(lhs.getZ(), rhs.getZ()) &&
         almostEqual(lhs.getW(), rhs.getW());
}

bool operator!=(const Quaternion& lhs, const Quaternion& rhs) {
  return !(lhs == rhs);
}

Quaternion operator+(const Quaternion& lhs, const Quaternion& rhs) {
  return Quaternion(lhs * rhs);
}

Quaternion operator-(const Quaternion& lhs, const Quaternion& rhs) {
  return Quaternion(lhs * rhs.conjugate());
}

Quaternion operator*(const Quaternion& lhs, const Quaternion& rhs) {
  return Quaternion(lhs.getW() * rhs.getX() + lhs.getX() * rhs.getW() +
                        lhs.getY() * rhs.getZ() - lhs.getZ() * rhs.getY(),
                    lhs.getW() * rhs.getY() + lhs.getY() * rhs.getW() +
                        lhs.getZ() * rhs.getX() - lhs.getX() * rhs.getZ(),
                    lhs.getW() * rhs.getZ() + lhs.getZ() * rhs.getW() +
                        lhs.getX() * rhs.getY() - lhs.getY() * rhs.getX(),
                    lhs.getW() * rhs.getW() - lhs.getX() * rhs.getX() -
                        lhs.getY() * rhs.getY() - lhs.getZ() * rhs.getZ());
}

// Conjugate
Quaternion Quaternion::conjugate() const {
  return Quaternion(-x_, -y_, -z_, w_);
}

// Conversion to euler
Euler Quaternion::toEuler() const {
  Euler euler;
  euler.setR(std::atan2(2 * (w_ * x_ + y_ * z_),
                        (w_ * w_ + z_ * z_ - x_ * x_ - y_ * y_)));
  euler.setP(std::asin(2 * (w_ * y_ - x_ * z_)));
  euler.setY(std::atan2(2 * (w_ * z_ + x_ * y_),
                        (w_ * w_ + x_ * x_ - y_ * y_ - z_ * z_)));
  return euler;
}

// Constructors and destructor
Pose::Pose() : position_(), orientation_() {}
Pose::Pose(const Vector3& position, const Quaternion& orientation)
    : position_(position), orientation_(orientation) {}
Pose::~Pose() {}

// Copy constructor and assignment operators
Pose::Pose(const Pose& other)
    : position_(other.getPos()), orientation_(other.getOri()) {}

Pose& Pose::operator=(const Pose& other) {
  position_ = other.getPos();
  orientation_ = other.getOri();
  return *this;
}

// Set and get data
Vector3 Pose::getPos() const { return position_; }
Quaternion Pose::getOri() const { return orientation_; }
void Pose::setPos(const Vector3& pos) { position_ = pos; }
void Pose::setOri(const Quaternion& ori) { orientation_ = ori; }
void Pose::setAbsPos() { position_.setAbs(); }

// Overload operators
bool operator==(const Pose& lhs, const Pose& rhs) {
  return lhs.getPos() == rhs.getPos() && lhs.getOri() == rhs.getOri();
}

bool operator!=(const Pose& lhs, const Pose& rhs) { return !(lhs == rhs); }

Pose operator+(const Pose& lhs, const Pose& rhs) {
  return Pose(lhs.getPos() + rhs.getPos(), lhs.getOri() + rhs.getOri());
}

Pose operator-(const Pose& lhs, const Pose& rhs) {
  return Pose(lhs.getPos() - rhs.getPos(), lhs.getOri() - rhs.getOri());
}

}  // namespace ground_truth

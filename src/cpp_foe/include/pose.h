#pragma once

#include <algorithm>
#include <cmath>
#include <vector>
#include <ros/ros.h>

namespace ground_truth {

// Forward declaration of all the classes
class Coordinates;
class Vector3;
class Euler;
class Quaternion;
class Pose;

bool almostEqual(const float& x, const float& y);

float dot(const Vector3& lhs, const Vector3& rhs);

Vector3 cross(const Vector3& lhs, const Vector3& rhs);

class Coordinates {
 public:
  Coordinates(const int& x, const int& y);
  ~Coordinates();

  int getX() const;
  int getY() const;

 private:
  const int x_;
  const int y_;
};

class Vector3 {
 public:
  Vector3();
  Vector3(const float& x, const float& y, const float& z);
  ~Vector3();

  // Copy constructor and assignment operators
  Vector3(const Vector3& other);
  Vector3& operator=(const Vector3& other);

  // Set and get data
  float getX() const;
  float getY() const;
  float getZ() const;
  void setX(const float& x);
  void setY(const float& y);
  void setZ(const float& z);
  void setAbs();
  float getModule() const;

  // Overload operators
  friend bool operator==(const Vector3& lhs, const Vector3& rhs);
  friend bool operator!=(const Vector3& lhs, const Vector3& rhs);
  friend Vector3 operator+(const Vector3& lhs, const Vector3& rhs);
  friend Vector3 operator-(const Vector3& lhs, const Vector3& rhs);
  // friend float operator*(const Vector3& lhs, const Vector3& rhs);
  friend Vector3 operator*(const float& lhs, const Vector3& rhs);
  friend Vector3 operator*(const Quaternion& lhs, const Vector3& rhs);
  friend Vector3 operator/(const Vector3& lhs, const float& rhs);

 private:
  float x_;
  float y_;
  float z_;
};

class Euler {
 public:
  Euler();
  Euler(const float& r, const float& p, const float& y);
  ~Euler();

  float getR() const;
  float getP() const;
  float getY() const;
  void setR(const float& r);
  void setP(const float& p);
  void setY(const float& y);

  Vector3 toVector() const;

 private:
  float r_;
  float p_;
  float y_;
};

class Quaternion {
 public:
  Quaternion();
  Quaternion(const float& x, const float& y, const float& z, const float& w);
  Quaternion(const Euler& euler);
  ~Quaternion();

  // Copy constructor and assignment operators
  Quaternion(const Quaternion& other);
  Quaternion& operator=(const Quaternion& other);

  // Set and get data
  float getX() const;
  float getY() const;
  float getZ() const;
  float getW() const;
  void setX(const float& x);
  void setY(const float& y);
  void setZ(const float& z);
  void setW(const float& w);

  // Overload operators
  friend bool operator==(const Quaternion& lhs, const Quaternion& rhs);
  friend bool operator!=(const Quaternion& lhs, const Quaternion& rhs);
  friend Quaternion operator+(const Quaternion& lhs, const Quaternion& rhs);
  friend Quaternion operator-(const Quaternion& lhs, const Quaternion& rhs);
  friend Quaternion operator*(const Quaternion& lhs, const Quaternion& rhs);

  // Conjugate
  Quaternion conjugate() const;

  // Conversion to euler
  Euler toEuler() const;

 private:
  float x_;
  float y_;
  float z_;
  float w_;
};

class Pose {
 public:
  Pose();
  Pose(const Vector3& position, const Quaternion& Quaternion);
  ~Pose();

  // Copy constructor and assignment operators
  Pose(const Pose& other);
  Pose& operator=(const Pose& other);

  // Set and get data
  Vector3 getPos() const;
  Quaternion getOri() const;
  void setPos(const Vector3& pos);
  void setOri(const Quaternion& ori);
  void setAbsPos();

  // Overload operators
  friend bool operator==(const Pose& lhs, const Pose& rhs);
  friend bool operator!=(const Pose& lhs, const Pose& rhs);
  friend Pose operator+(const Pose& lhs, const Pose& rhs);
  friend Pose operator-(const Pose& lhs, const Pose& rhs);

 private:
  Vector3 position_;
  Quaternion orientation_;
};

}  // namespace relative_localization
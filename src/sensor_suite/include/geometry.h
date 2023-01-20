typedef struct {
  int u, v;
} pixel;
typedef struct {
  float x, y, z;
} vector;

typedef struct {
  vector origin;
  vector dir;
} ray;

static inline vector newVector(float x, float y, float z) {
  vector v;
  v.x = x;
  v.y = y;
  v.z = z;
  return v;
}

static inline vector copyVector(vector v1) {
  vector v2;
  v2.x = v1.x;
  v2.y = v1.y;
  v2.z = v1.z;
  return v2;
}

static inline ray newRay(vector origin, vector dir) {
  ray r;
  r.origin = origin;
  r.dir = dir;
  return r;
}

static inline vector qsubtract(vector v1, vector v2) {
  vector v;
  v.x = (float)((double)v1.x - (double)v2.x);
  v.y = (float)((double)v1.y - (double)v2.y);
  v.z = (float)((double)v1.z - (double)v2.z);
  return v;
}

static inline float qdot(vector v1, vector v2) {
  float x = v1.x * v2.x;
  float y = v1.y * v2.y;
  float z = v1.z * v2.z;
  return (float)((double)x + (double)y + (double)z);
}

static inline vector qcross(vector v1, vector v2) {
  vector v;
  v.x = (float)((double)(v1.y * v2.z) - (double)(v1.z * v2.y));
  v.y = (float)((double)(v1.z * v2.x) - (double)(v1.x * v2.z));
  v.z = (float)((double)(v1.x * v2.y) - (double)(v1.y * v2.x));
  return v;
}

static inline float qsize(vector v) {
  float x = v.x * v.x;
  float y = v.y * v.y;
  float z = v.z * v.z;
  return sqrt((float)((double)x + (double)y + (double)z));
}

static inline float qnorm_sq(vector v) {
  float x = v.x * v.x;
  float y = v.y * v.y;
  float z = v.z * v.z;
  return (float)((double)x + (double)y + (double)z);
}

static inline vector scale(float c, vector v1) {
  vector v;
  v.x = v1.x * c;
  v.y = v1.y * c;
  v.z = v1.z * c;
  return v;
}

static inline vector qadd(vector v1, vector v2) {
  vector v;
  v.x = (float)((double)v1.x + (double)v2.x);
  v.y = (float)((double)v1.y + (double)v2.y);
  v.z = (float)((double)v1.z + (double)v2.z);
  return v;
}

static inline float qdist(vector v1, vector v2) {
  double f = ((double)v1.x - (double)v2.x) * ((double)v1.x - (double)v2.x);
  f += ((double)v1.y - (double)v2.y) * ((double)v1.y - (double)v2.y);
  f += ((double)v1.z - (double)v2.z) * ((double)v1.z - (double)v2.z);
  return sqrt((float)f);
}

static inline int equals(vector v1, vector v2) {
  if (v1.x != v2.x) return 0;
  if (v1.y != v2.y) return 0;
  if (v1.z != v2.z) return 0;
  return 1;
}

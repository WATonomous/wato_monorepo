# Current Time Complexities for Bounding Box Computations

## minAreaRect
- typically implemented as covex hull (O(Nl log N)) + rotating calipers (O(H))

The issues with this is that it is extremely sensitive to outliers. minAreaRect depends on the convex hull, so a single outlier can rotate/expand the hull and skew orientation sizes. Its hard to perfectly have the clusters be associated to a certain object, though parameters could use some tuning. 

The minimal-area rectangle encloses points, but its heading may not match an objects's true longitudinal axis under partial views, L-shapes, or irregular hulls 

PCA in (2D):
- O(N) to accumulate covariance + O(1) for 2x2 eigensolve + O(N) for min/max on eigen axes; overall linear time and constant memory

- for ground-contact vehicles, the quantity you care about is the footprint yaw on the horizontal plane. 2D PCA on XY directly estimates the dominant heading of that foorprint, not just the hull

- 2D PCA average all inliers, so its more stable than minAreaRect, which can swing with hull outliers or partial views

- cheaper to compute than minAreaRect

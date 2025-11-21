# Suggestions for Favoring Rectangular Clusters in Bounding Box Orientation

## Problem
When clusters merge (e.g., car + traffic island), the resulting bounding box can become sideways because:
1. The L-shape fit finds minimum area rectangle, which may not match vehicle orientation
2. The 90° ambiguity resolution only uses line-of-sight, ignoring shape characteristics
3. Merged clusters may not be rectangular, but the algorithm still tries to fit a rectangle

## Solution Approaches

### 1. **Add Aspect Ratio Penalty to Orientation Selection** (Recommended - Easy to implement)

Modify the candidate selection in `computeClusterBox` to favor orientations with realistic vehicle aspect ratios (typically 2:1 to 5:1 length:width).

**Implementation:**
- Add an aspect ratio score to each candidate
- Penalize candidates that are too square (aspect ratio < 1.5) or too elongated (aspect ratio > 6.0)
- Combine angle difference with aspect ratio penalty

**Code changes needed in `computeClusterBox`:**

```cpp
// After computing candidates, add aspect ratio scoring:
struct Candidate {
  double yaw;
  double len;
  double wid;
  double diff;           // angle difference from LOS
  double aspect_ratio;   // len/wid
  double score;          // combined score
};

// Constants for aspect ratio scoring
const double min_aspect_ratio = 1.5;   // Minimum realistic aspect ratio (e.g., wide truck)
const double max_aspect_ratio = 6.0;   // Maximum realistic aspect ratio (e.g., long truck)
const double ideal_aspect_ratio = 3.0; // Typical car aspect ratio
const double aspect_ratio_weight = 0.3; // Weight for aspect ratio vs angle difference

// Compute aspect ratio and score for each candidate
for (auto& cand : cands) {
  cand.aspect_ratio = cand.len / std::max(cand.wid, 0.1); // Avoid division by zero
  
  // Penalize aspect ratios outside realistic range
  double aspect_penalty = 0.0;
  if (cand.aspect_ratio < min_aspect_ratio) {
    // Too square - penalize heavily
    aspect_penalty = (min_aspect_ratio - cand.aspect_ratio) * 2.0;
  } else if (cand.aspect_ratio > max_aspect_ratio) {
    // Too elongated - penalize
    aspect_penalty = (cand.aspect_ratio - max_aspect_ratio) * 1.0;
  } else {
    // Reward aspect ratios closer to ideal
    double deviation = std::abs(cand.aspect_ratio - ideal_aspect_ratio);
    aspect_penalty = deviation * 0.1; // Small penalty for deviation from ideal
  }
  
  // Normalize angle difference to [0, 1] range (max difference is PI)
  double normalized_angle_diff = cand.diff / M_PI;
  
  // Combined score: lower is better
  cand.score = (1.0 - aspect_ratio_weight) * normalized_angle_diff + 
               aspect_ratio_weight * aspect_penalty;
}

// Pick candidate with best (lowest) combined score
Candidate best = cands[0];
for (size_t i = 1; i < cands.size(); ++i) {
  if (cands[i].score < best.score) {
    best = cands[i];
  }
}
```

### 2. **Add Rectangularity Score Based on Point Distribution** (Medium complexity)

Compute how well the cluster points fit a rectangular shape by checking point distribution along length vs width.

**Implementation:**
- For each candidate orientation, project points to the candidate's local frame
- Check if points are distributed along the length axis (good) vs width axis (bad)
- Penalize candidates where points are evenly distributed (suggests non-rectangular shape)

**Code snippet:**

```cpp
// Helper function to compute rectangularity score
double computeRectangularityScore(
    const pcl::PointCloud<pcl::PointXYZ>& cloud,
    const std::vector<int>& indices,
    double yaw, double len, double wid,
    const Eigen::Vector2f& center) {
  
  if (indices.empty() || len < 0.1 || wid < 0.1) return 0.0;
  
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);
  
  // Project points to local frame (aligned with candidate orientation)
  double length_variance = 0.0;
  double width_variance = 0.0;
  double mean_length = 0.0;
  double mean_width = 0.0;
  
  for (int idx : indices) {
    const auto& pt = cloud.points[idx];
    // Translate to center
    double dx = pt.x - center.x();
    double dy = pt.y - center.y();
    
    // Rotate to local frame
    double local_x = dx * cos_yaw + dy * sin_yaw;  // along length
    double local_y = -dx * sin_yaw + dy * cos_yaw; // along width
    
    mean_length += local_x;
    mean_width += local_y;
  }
  
  mean_length /= indices.size();
  mean_width /= indices.size();
  
  // Compute variance
  for (int idx : indices) {
    const auto& pt = cloud.points[idx];
    double dx = pt.x - center.x();
    double dy = pt.y - center.y();
    double local_x = dx * cos_yaw + dy * sin_yaw;
    double local_y = -dx * sin_yaw + dy * cos_yaw;
    
    length_variance += (local_x - mean_length) * (local_x - mean_length);
    width_variance += (local_y - mean_width) * (local_y - mean_width);
  }
  
  length_variance /= indices.size();
  width_variance /= indices.size();
  
  // Good rectangularity: high variance along length, low variance along width
  // Score = length_variance / (width_variance + 0.1) - higher is better
  return length_variance / (width_variance + 0.1);
}

// Use in candidate selection:
for (auto& cand : cands) {
  double rect_score = computeRectangularityScore(
    *cloud, cluster.indices, cand.yaw, cand.len, cand.wid, 
    lshape_result.center_xy);
  
  // Normalize rectangularity score (typically 0-10 range)
  double normalized_rect = std::min(rect_score / 10.0, 1.0);
  
  // Add to candidate score (invert so lower is better)
  cand.score += (1.0 - normalized_rect) * 0.2; // 20% weight
}
```

### 3. **Improve Merge Logic to Check Rectangularity** (Medium complexity)

Before merging clusters, check if the merged cluster would be rectangular. If not, don't merge.

**Implementation:**
- Before merging, compute a quick L-shape fit on the hypothetical merged cluster
- Check aspect ratio and rectangularity
- Only merge if the result would be reasonably rectangular

**Code changes in `mergeClusters`:**

```cpp
void ProjectionUtils::mergeClusters(std::vector<pcl::PointIndices>& cluster_indices,
                                    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                    double mergeTolerance) {
  if (cloud->empty() || cluster_indices.empty()) return;

  std::vector<bool> merged(cluster_indices.size(), false);

  for (size_t i = 0; i < cluster_indices.size(); ++i) {
    if (merged[i]) continue;

    Eigen::Vector4f centroid_i;
    pcl::compute3DCentroid(*cloud, cluster_indices[i].indices, centroid_i);

    for (size_t j = i + 1; j < cluster_indices.size(); ++j) {
      if (merged[j]) continue;

      Eigen::Vector4f centroid_j;
      pcl::compute3DCentroid(*cloud, cluster_indices[j].indices, centroid_j);

      double distance = (centroid_i - centroid_j).norm();

      if (distance < mergeTolerance) {
        // NEW: Check if merged cluster would be rectangular
        std::vector<int> merged_indices = cluster_indices[i].indices;
        merged_indices.insert(merged_indices.end(),
                            cluster_indices[j].indices.begin(),
                            cluster_indices[j].indices.end());
        
        // Quick L-shape fit check
        LShapeResult test_fit = computeLShapeFit(*cloud, merged_indices);
        if (test_fit.ok) {
          double aspect_ratio = test_fit.len / std::max(test_fit.wid, 0.1f);
          
          // Only merge if aspect ratio is reasonable (1.2 to 7.0)
          // This prevents merging car + traffic island (which would be too square)
          if (aspect_ratio >= 1.2 && aspect_ratio <= 7.0) {
            cluster_indices[i].indices = std::move(merged_indices);
            merged[j] = true;
          }
          // If aspect ratio is bad, skip merging
        } else {
          // If L-shape fit fails, don't merge
        }
      }
    }
  }

  // Remove merged clusters from the list
  std::vector<pcl::PointIndices> filtered_clusters;
  for (size_t i = 0; i < cluster_indices.size(); ++i) {
    if (!merged[i]) {
      filtered_clusters.push_back(cluster_indices[i]);
    }
  }
  cluster_indices = filtered_clusters;
}
```

### 4. **Add L-Shape Fit Quality Check** (Easy to implement)

Reject L-shape fits that are too square or have poor fit quality.

**Code changes in `computeClusterBox`:**

```cpp
LShapeResult lshape_result = computeLShapeFit(*cloud, cluster.indices);
if (lshape_result.ok) {
  // Check aspect ratio - reject if too square (likely merged objects)
  double aspect_ratio = lshape_result.len / std::max(lshape_result.wid, 0.1f);
  
  // Typical vehicles have aspect ratio 2.0-5.0
  // If aspect ratio is < 1.3, it's likely a merged cluster (car + obstacle)
  if (aspect_ratio < 1.3) {
    // Fall back to axis-aligned bounding box
    lshape_result.ok = false;
  }
}

if (lshape_result.ok) {
  // ... existing orientation selection code ...
} else {
  // Fallback: axis-aligned in XY
  // ... existing fallback code ...
}
```

### 5. **Add Configuration Parameters** (Easy to implement)

Make aspect ratio thresholds configurable via params.yaml:

```yaml
spatial_association:
  ros__parameters:
    # ... existing parameters ...
    
    # Bounding box orientation parameters
    orientation_params:
      min_aspect_ratio: 1.5      # Minimum length:width ratio for valid orientation
      max_aspect_ratio: 6.0       # Maximum length:width ratio for valid orientation
      ideal_aspect_ratio: 3.0     # Ideal aspect ratio (typical car)
      aspect_ratio_weight: 0.3    # Weight for aspect ratio vs angle difference (0-1)
      rectangularity_weight: 0.2  # Weight for rectangularity score (0-1)
      check_merge_rectangularity: true  # Check rectangularity before merging
```

## Recommended Implementation Order

1. **Start with #1 (Aspect Ratio Penalty)** - Easiest, biggest impact
2. **Add #4 (L-Shape Fit Quality Check)** - Quick win, prevents bad fits
3. **Add #3 (Improve Merge Logic)** - Prevents problematic merges
4. **Add #2 (Rectangularity Score)** - Fine-tuning for edge cases
5. **Add #5 (Configuration)** - Make it tunable

## Testing

After implementing, test with:
- Close vehicles (where merging is more likely)
- Vehicles near traffic islands/barriers
- Parked vehicles
- Different vehicle types (cars, trucks, buses)

Monitor:
- Aspect ratios of detected boxes (should be 2-5 for vehicles)
- Orientation accuracy (should align with vehicle heading)
- False positives (should decrease)


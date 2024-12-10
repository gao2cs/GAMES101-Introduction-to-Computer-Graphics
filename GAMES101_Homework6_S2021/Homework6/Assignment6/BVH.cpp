#include <algorithm>
#include <cassert>
#include "BVH.hpp"

BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
                   SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
      primitives(std::move(p))
{
    time_t start, stop;
    time(&start);
    if (primitives.empty())
        return;

    if (splitMethod == SplitMethod::NAIVE) {
        root = recursiveBuild(primitives);  
    }

    if (splitMethod == BVHAccel::SplitMethod::SAH) {
            root = recursiveSVHBuild(primitives); 
    }

    time(&stop);
    double diff = difftime(stop, start);
    int hrs = (int)diff / 3600;
    int mins = ((int)diff / 60) - (hrs * 60);
    int secs = (int)diff - (hrs * 3600) - (mins * 60);

    printf(
        "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
        hrs, mins, secs);
}


// *** Implementation of SAH follows pseudo code strictly from: http://15362.courses.cs.cmu.edu/fall2024/lecture/lecture-09 ********
// The SAH is much faster than BVH when triangles are more spread out! Tested example with a sorter.
// My implementation is neither optimized nor necessarily correct, however already much better than BVH => SAH is doing something.
class CBin {
public:
    // Constructor to initialize the bin
    CBin() : bounds(Bounds3()), count(0) {}

    void addObject(Object* obj) {
        bounds = Union(bounds, obj->getBounds()); 
        count++; 
    }

    void clear() {
        bounds = Bounds3(); 
        count = 0;          
    }

    Bounds3 bounds; 
    int count;      
};

BVHBuildNode* BVHAccel::recursiveSVHBuild(std::vector<Object*> objects) {
    BVHBuildNode* node = new BVHBuildNode();

    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());

    if (objects.size() == 1) {
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if (objects.size() == 2) {
        node->left = recursiveSVHBuild(std::vector{ objects[0] });
        node->right = recursiveSVHBuild(std::vector{ objects[1] });
        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else {
        int bin_n = 32;
        float best_cost = std::numeric_limits<float>::infinity();
        int best_partition = -1;
        int best_axis = -1;

        // Repeat for each of the three axes: Sort objects based on their centroid along axis
        for (int axis = 0; axis < 3; ++axis) {
            std::vector<CBin> bins(bin_n); // Reset bins for each axis

            // Compute bounds along the current axis
            Bounds3 centroidBounds;
            for (const auto& obj : objects)
                centroidBounds = Union(centroidBounds, obj->getBounds().Centroid());

            float boundsMin = (axis == 0) ? centroidBounds.pMin.x : (axis == 1) ? centroidBounds.pMin.y : centroidBounds.pMin.z;
            float boundsMax = (axis == 0) ? centroidBounds.pMax.x : (axis == 1) ? centroidBounds.pMax.y : centroidBounds.pMax.z;
            float binSize = (boundsMax - boundsMin) / bin_n;

            std::vector<Object*> sortedObjects = objects; 
            std::sort(sortedObjects.begin(), sortedObjects.end(), [axis](auto f1, auto f2) {
                if (axis == 0)
                    return f1->getBounds().Centroid().x < f2->getBounds().Centroid().x;
                else if (axis == 1)
                    return f1->getBounds().Centroid().y < f2->getBounds().Centroid().y;
                else
                    return f1->getBounds().Centroid().z < f2->getBounds().Centroid().z;
                });

            // Assign sorted objects to bins
            for (int i = 0; i < sortedObjects.size(); ++i) {
                float centroid = (axis == 0) ? sortedObjects[i]->getBounds().Centroid().x
                    : (axis == 1) ? sortedObjects[i]->getBounds().Centroid().y
                    : sortedObjects[i]->getBounds().Centroid().z;
                int binIndex = std::clamp(int((centroid - boundsMin) / binSize), 0, bin_n - 1);
                bins[binIndex].addObject(sortedObjects[i]);
            }

            // Compute SAH for this axis
            int totalCount = 0;
            for (int i = 0; i < bin_n; ++i)
                totalCount += bins[i].count;

            int leftCount = 0;
            int rightCount = totalCount;

            for (int j = 1; j < bin_n; ++j) {
                leftCount += bins[j - 1].count;
                rightCount -= bins[j - 1].count;

                Bounds3 a, b;
                for (int i = 0; i < j; ++i)
                    a = Union(a, bins[i].bounds);
                for (int i = j; i < bin_n; ++i)
                    b = Union(b, bins[i].bounds);

                // C' = SA * NA + SB * NB
                float cost = a.SurfaceArea() * leftCount + b.SurfaceArea() * rightCount;

                // Update min cost parameters
                if (cost < best_cost) {
                    best_cost = cost;
                    best_partition = leftCount; // Stands for the (leftCount)th object to parition along best_axis
                    best_axis = axis;
                }
            }
        }

        // Sort on the real objects according to the best_axis to be compatible with best_partition later
        std::sort(objects.begin(), objects.end(), [best_axis](auto f1, auto f2) {
            if (best_axis == 0)
                return f1->getBounds().Centroid().x < f2->getBounds().Centroid().x;
            else if (best_axis == 1)
                return f1->getBounds().Centroid().y < f2->getBounds().Centroid().y;
            else
                return f1->getBounds().Centroid().z < f2->getBounds().Centroid().z;
            });


        auto beginning = objects.begin();
        auto middling = objects.begin() + best_partition; // best_partition corresponding to the minimum sort considering all three axes x, y, and z

        auto leftshapes = std::vector<Object*>(beginning, middling);
        auto rightshapes = std::vector<Object*>(middling, objects.end());

        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        node->left = recursiveSVHBuild(leftshapes);
        node->right = recursiveSVHBuild(rightshapes);

        node->bounds = Union(node->left->bounds, node->right->bounds);
    }

    return node;
} // ********************Done Implementing SAH *********************************



// Refer to Games101 Slides 14 for Building BVHs
// 1. Find the centroid bounding box for all the objects in the node
// 2. Choose the longest dim corresponding to the max len of the bounding box in the node to split
// 3. Sort the objects in the node along this axis and split at the location of the median object
// Note: The bounding box is the union of the left and right shapes for the node
BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());
    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if (objects.size() == 2) {
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else {
        
        // The centroid bounding box containing the centroids of all the objects in the scene. Note: centroid bounding box != bounding box of the objects
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds =
                Union(centroidBounds, objects[i]->getBounds().Centroid());
        // The dim corresponding to the maximum length of the centroid bounding box(0: x_axis, 1:y_axis & 2:z_axis) 
        int dim = centroidBounds.maxExtent();

        // Sort the storing order of the objects by the one dimensional centroid coordinate: x in distance
        switch (dim) {
        case 0:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().x <
                       f2->getBounds().Centroid().x;
            });
            break;
        case 1:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().y <
                       f2->getBounds().Centroid().y;
            });
            break;
        case 2:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().z <
                       f2->getBounds().Centroid().z;
            });
            break;
        }

        // Iterators in C++ are right exclusive [start, end)
        // Suppose [A, B, C, D, E, F, G, H, I, J]. Then size = 10 so size / 2 = 5.
        // leftshapes: [A, B, C, D, E]
        // rightshapes: [F, G, H, I, J]
        auto beginning = objects.begin();
        auto middling = objects.begin() + (objects.size() / 2);
        auto ending = objects.end();
        
        // Range constructors [beginning, middling)
        auto leftshapes = std::vector<Object*>(beginning, middling);
        // Range constructors [middling, ending)
        auto rightshapes = std::vector<Object*>(middling, ending);

        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        node->left = recursiveBuild(leftshapes);
        node->right = recursiveBuild(rightshapes);

        node->bounds = Union(node->left->bounds, node->right->bounds);
    }

    return node;
}

Intersection BVHAccel::Intersect(const Ray& ray) const
{
    Intersection isect;
    if (!root)
        return isect;
    isect = BVHAccel::getIntersection(root, ray);
    return isect;
}

// Assuming that BVH has been built based on the scene objects.
// Now, follow exactly the pseudo code in games101 Lec14 - Slide 37 for BVH traversal to search for the closet hit triangle.
// 1. Every node has a bounding box defined
// 2. Only leaf node can store objects
// 3. Want triangle with the closest hit stored as and returned as "Intersection"
Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const // For each ray, ...
{
    // TODO Traverse the BVH to find intersection
    Intersection intersection;
    if (node == nullptr || !node->bounds.IntersectP(ray, ray.direction_inv, { ray.direction.x > 0, ray.direction.y > 0, ray.direction.z > 0 })) {
        return intersection; // NULL intersection
    }

    if (node->left == nullptr && node->right == nullptr) {
       return node->object->getIntersection(ray);
    }

    Intersection hit1 = getIntersection(node->left, ray);  // closest hit from the left subtree
    Intersection hit2 = getIntersection(node->right, ray); // closest hit from the right subtree

    return hit1.distance < hit2.distance ? hit1 : hit2; // Return minimum distance hit triangle
}
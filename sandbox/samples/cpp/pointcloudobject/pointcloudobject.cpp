
//
// This source file is part of appleseed.
// Visit http://appleseedhq.net/ for additional information and resources.
//
// This software is released under the MIT license.
//
// Copyright (c) 2017 Francois Beaune, The appleseedhq Organization
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//

#include "pointcloudobject.h"

// appleseed.renderer headers.
#include "renderer/api/object.h"
#include "renderer/api/project.h"
#include "renderer/api/rendering.h"
#include "renderer/api/scene.h"
#include "renderer/api/types.h"

// todo: fix.
#include "renderer/kernel/shading/shadingray.h"

// appleseed.foundation headers.
#include "foundation/math/hash.h"
#include "foundation/math/intersection/rayaabb.h"
#include "foundation/math/ray.h"
#include "foundation/math/rng/distribution.h"
#include "foundation/math/rng/xoroshiro128plus.h"
#include "foundation/math/scalar.h"
#include "foundation/math/vector.h"
#include "foundation/platform/types.h"
#include "foundation/utility/api/specializedapiarrays.h"
#include "foundation/utility/containers/dictionary.h"
#include "foundation/utility/job/iabortswitch.h"
#include "foundation/utility/casts.h"
#include "foundation/utility/searchpaths.h"
#include "foundation/utility/string.h"

// appleseed.main headers.
#include "main/dllvisibility.h"

// Standard headers.
#include <algorithm>
#include <cmath>
#include <cstddef>

namespace
{
    //
    // PointCloudObject class implementation.
    //

    const char* Model = "point_cloud_object";

    class PointCloudObject
      : public asr::ProceduralObject
    {
      public:
        // Constructor.
        PointCloudObject(
            const char*                 name,
            const asr::ParamArray&      params)
          : asr::ProceduralObject(name, params)
          , m_lazy_region_kit(&m_region_kit)
          , m_bbox_min(0.0f, 0.0f, 0.0f)
          , m_bbox_max(0.0f, 0.0f, 0.0f)
        {
            // Define a cube.
            m_points.push_back(asf::Vector3f(-0.5f, -0.5f, -0.5f));
            m_points.push_back(asf::Vector3f(-0.3f, -0.5f, -0.5f));
            m_points.push_back(asf::Vector3f( 0.3f, -0.5f, -0.5f));
            m_points.push_back(asf::Vector3f( 0.5f,  0.5f,  0.3f));
            m_points.push_back(asf::Vector3f( 0.5f,  0.5f, -0.3f));
            m_points.push_back(asf::Vector3f( 0.5f,  0.5f, -0.5f));
            m_points.push_back(asf::Vector3f( 0.5f, -0.5f, -0.5f));
            m_points.push_back(asf::Vector3f( 0.5f, -0.3f, -0.5f));
            m_points.push_back(asf::Vector3f( 0.5f,  0.3f, -0.5f));
            m_points.push_back(asf::Vector3f( 0.5f, -0.5f,  0.5f));
            m_points.push_back(asf::Vector3f( 0.5f, -0.3f,  0.5f));
            m_points.push_back(asf::Vector3f( 0.5f,  0.3f,  0.5f));
            m_points.push_back(asf::Vector3f(-0.5f,  0.5f,  0.5f));
            m_points.push_back(asf::Vector3f(-0.5f, -0.5f,  0.5f));
            m_points.push_back(asf::Vector3f(-0.3f, -0.5f,  0.5f));
            m_points.push_back(asf::Vector3f( 0.3f, -0.5f,  0.5f));
            m_points.push_back(asf::Vector3f(-0.5f,  0.5f, -0.5f));
            m_points.push_back(asf::Vector3f(-0.5f,  0.5f, -0.3f));
            m_points.push_back(asf::Vector3f(-0.5f,  0.5f,  0.3f));
            m_points.push_back(asf::Vector3f( 0.5f,  0.5f,  0.5f));
            
            // Compute point cloud bbox.
            for(size_t i = 0; i < m_points.size(); ++i)
            {
                if(m_points[i][0] < m_bbox_min[0])
                    m_bbox_min[0] = m_points[i][0];
                else if(m_points[i][0] > m_bbox_max[0])
                    m_bbox_max[0] = m_points[i][0];
                
                if(m_points[i][1] < m_bbox_min[1])
                    m_bbox_min[1] = m_points[i][1];
                else if(m_points[i][1] > m_bbox_max[1])
                    m_bbox_max[1] = m_points[i][1];
                
                if(m_points[i][2] < m_bbox_min[2])
                    m_bbox_min[2] = m_points[i][2];
                else if(m_points[i][2] > m_bbox_max[2])
                    m_bbox_max[2] = m_points[i][2];
            }

            // Add tolerance to bbox.
            const float Epsilon = 0.1f;
            m_bbox_min[0] = m_bbox_min[0] - Epsilon;
            m_bbox_min[1] = m_bbox_min[1] - Epsilon;
            m_bbox_min[2] = m_bbox_min[2] - Epsilon;
            m_bbox_max[0] = m_bbox_max[0] + Epsilon;
            m_bbox_max[1] = m_bbox_max[1] + Epsilon;
            m_bbox_max[2] = m_bbox_max[2] + Epsilon;
        }

        // Delete this instance.
        void release() override
        {
            delete this;
        }

        // Return a string identifying this object model.
        const char* get_model() const override
        {
            return Model;
        }

        // Compute the local space bounding box of the object over the shutter interval.
        asr::GAABB3 compute_local_bbox() const override
        {
            return asr::GAABB3(asr::GVector3(-5.0f), asr::GVector3(5.0f));
        }

        // Return the region kit of the object.
        asf::Lazy<asr::RegionKit>& get_region_kit() override
        {
            return m_lazy_region_kit;
        }

        // Access materials slots.
        size_t get_material_slot_count() const override
        {
            return 1;
        }
        const char* get_material_slot(const size_t index) const override
        {
            return "default";
        }

        // Compute the intersection between a ray expressed in object space and
        // the surface of this object and return detailed intersection results.
        void intersect(
            const asr::ShadingRay&  ray,
            IntersectionResult&     result) const override
        {
            double t;
            asf::Vector3d p;
            result.m_hit = raymarch(ray, 4, t, p);

            if (result.m_hit)
            {
                result.m_distance = t;

                const float H = 1.0e-4f;

                asf::Vector3f n(
                    evaluate_field(p.x + H, p.y, p.z) - evaluate_field(p.x - H, p.y, p.z),
                    evaluate_field(p.x, p.y + H, p.z) - evaluate_field(p.x, p.y - H, p.z),
                    evaluate_field(p.x, p.y, p.z + H) - evaluate_field(p.x, p.y, p.z - H));
                n = asf::normalize(n);

                result.m_geometric_normal = asf::Vector3d(n);
                result.m_shading_normal = asf::Vector3d(n);

                result.m_uv = asf::Vector2f(0.0f);
                result.m_material_slot = 0;
            }
        }

        // Compute the intersection between a ray expressed in object space and
        // the surface of this object and simply return whether there was a hit.
        bool intersect(
            const asr::ShadingRay&  ray) const override
        {
            double t;
            asf::Vector3d p;
            return raymarch(ray, 4, t, p);
        }

      private:
        asr::RegionKit              m_region_kit;
        asf::Lazy<asr::RegionKit>   m_lazy_region_kit;

        std::vector<asf::Vector3f>  m_points;
        asf::Vector3f               m_bbox_min;
        asf::Vector3f               m_bbox_max;
        //
        // Signed distance function.
        //
        // References:
        //
        //   http://iquilezles.org/www/articles/distfunctions/distfunctions.htm
        //
        //   http://blog.hvidtfeldts.net/index.php/2011/09/distance-estimated-3d-fractals-v-the-mandelbulb-different-de-approximations/
        //

        float evaluate_field(asf::Vector3f p) const
        {
            const bool inside = 
                p[0] >= m_bbox_min[0] && p[0] <= m_bbox_max[0] &&
                p[1] >= m_bbox_min[1] && p[1] <= m_bbox_max[1] &&
                p[2] >= m_bbox_min[2] && p[2] <= m_bbox_max[2];
            
            if(!inside)
                return 0.0f;

            const float threshold = 10.0;
            // Compute total field value, influenced by all the points.
            float field_value = 0;
            for(size_t i = 0; i < m_points.size(); ++i)
            {
                field_value +=  1.0 / asf::square_norm(p - m_points[i]);
            }
    
            // Threshold is the value we want to show.
            return field_value - threshold;
        }

        //
        // Modeling utilities.
        //

        static asf::Xoroshiro128plus make_rng(const asr::ShadingRay& ray)
        {
            return
                asf::Xoroshiro128plus(
                    asf::hash_uint64(asf::binary_cast<asf::uint64>(ray.m_org.x)) ^
                    asf::hash_uint64(asf::binary_cast<asf::uint64>(ray.m_org.y)) ^
                    asf::hash_uint64(asf::binary_cast<asf::uint64>(ray.m_org.z)),
                    asf::hash_uint64(asf::binary_cast<asf::uint64>(ray.m_dir.x)) ^
                    asf::hash_uint64(asf::binary_cast<asf::uint64>(ray.m_dir.y)) ^
                    asf::hash_uint64(asf::binary_cast<asf::uint64>(ray.m_dir.z)));
        }

        static float op_union(const float a, const float b)
        {
            return std::min(a, b);
        }

        static float op_substraction(const float a, const float b)
        {
            return std::max(a, -b);
        }

        static float op_intersection(const float a, const float b)
        {
            return std::max(a, b);
        }

        //
        // Raymarcher.
        //
        // References:
        //
        //   http://jamie-wong.com/2016/07/15/ray-marching-signed-distance-functions/
        //
        //   http://erleuchtet.org/~cupe/permanent/enhanced_sphere_tracing.pdf
        //

        float evaluate_field(const double x, const double y, const double z) const
        {
            return evaluate_field(asf::Vector3d(x, y, z));
        }

        float evaluate_field(const asf::Vector3d& p) const
        {
            return evaluate_field(asf::Vector3f(p));
        }

        
        bool raymarch(
            const asr::ShadingRay&  ray,
            const size_t            max_refinement_level,
            double&                 t_out,
            asf::Vector3d&          p_out) const
        {
            const auto bbox = asf::AABB3d(compute_local_bbox());
            asr::ShadingRay clipped_ray(ray);
            const asf::RayInfo3d clipped_ray_info(clipped_ray);
            if (!asf::clip(clipped_ray, clipped_ray_info, bbox))
                return false;
            asf::Xoroshiro128plus rng(
                asf::hash_uint64(asf::binary_cast<asf::uint64>(ray.m_org.x)) ^
                asf::hash_uint64(asf::binary_cast<asf::uint64>(ray.m_org.y)) ^
                asf::hash_uint64(asf::binary_cast<asf::uint64>(ray.m_org.z)),
                asf::hash_uint64(asf::binary_cast<asf::uint64>(ray.m_dir.x)) ^
                asf::hash_uint64(asf::binary_cast<asf::uint64>(ray.m_dir.y)) ^
                asf::hash_uint64(asf::binary_cast<asf::uint64>(ray.m_dir.z)));
            const size_t InitialStepCount = 100;
            size_t level = 1;
            double step_size = (clipped_ray.m_tmax - clipped_ray.m_tmin) / InitialStepCount;
            const double Epsilon = step_size;
            auto t = std::max(clipped_ray.m_tmin, Epsilon);
            const auto outside_sign = evaluate_field(ray.point_at(t)) > 0;
            t += asf::rand_double1(rng, 0.0, step_size);
            while (t < clipped_ray.m_tmax)
            {
                const auto p = clipped_ray.point_at(t);
                const auto sign = evaluate_field(p) > 0;
                if (sign != outside_sign)
                {
                    if (level == max_refinement_level)
                    {
                        t_out = t;
                        p_out = p;
                        return true;
                    }
                    else
                    {
                        t -= step_size;
                        step_size /= 10;
                        ++level;
                    }
                }
                t += step_size;
            }
            return false;
        }
    };
}

//
// PointCloudObjectFactory class implementation.
//

void PointCloudObjectFactory::release()
{
    delete this;
}

const char* PointCloudObjectFactory::get_model() const
{
    return Model;
}

asf::Dictionary PointCloudObjectFactory::get_model_metadata() const
{
    return
        asf::Dictionary()
            .insert("name", Model)
            .insert("label", "Point Cloud Object");
}

asf::DictionaryArray PointCloudObjectFactory::get_input_metadata() const
{
    asf::DictionaryArray metadata;
    return metadata;
}

asf::auto_release_ptr<asr::Object> PointCloudObjectFactory::create(
    const char*                 name,
    const asr::ParamArray&      params) const
{
    return asf::auto_release_ptr<asr::Object>(new PointCloudObject(name, params));
}

bool PointCloudObjectFactory::create(
    const char*                 name,
    const asr::ParamArray&      params,
    const asf::SearchPaths&     search_paths,
    const bool                  omit_loading_assets,
    asr::ObjectArray&           objects) const
{
    objects.push_back(create(name, params).release());
    return true;
}

//
// Plugin entry point.
//

extern "C"
{
    APPLESEED_DLL_EXPORT asr::IObjectFactory* appleseed_create_object_factory()
    {
        return new PointCloudObjectFactory();
    }
}

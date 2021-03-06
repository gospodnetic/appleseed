
//
// This source file is part of appleseed.
// Visit http://appleseedhq.net/ for additional information and resources.
//
// This software is released under the MIT license.
//
// Copyright (c) 2010-2013 Francois Beaune, Jupiter Jazz Limited
// Copyright (c) 2014-2017 Francois Beaune, The appleseedhq Organization
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

#ifndef APPLESEED_RENDERER_UTILITY_BBOX_H
#define APPLESEED_RENDERER_UTILITY_BBOX_H

// appleseed.foundation headers.
#include "foundation/math/scalar.h"

// Standard headers.
#include <cassert>
#include <cstddef>

namespace renderer
{

// Compute the bounding box of a set of objects using their get_parent_bbox() method.
template <typename BBox, typename Iterator>
BBox get_parent_bbox(const Iterator begin, const Iterator end);

// Compute the bounding box of a set of objects using their compute_parent_bbox() method.
template <typename BBox, typename Iterator>
BBox compute_parent_bbox(const Iterator begin, const Iterator end);

// Compute the union of a set of bounding boxes.
template <typename BBox, typename Iterator>
BBox compute_union(const Iterator begin, const Iterator end);

// Evaluate a path of equidistant bounding boxes for a given time value in [0,1).
template <typename BBox, typename Iterator>
BBox interpolate(const Iterator begin, const Iterator end, const double time);


//
// Implementation.
//

template <typename BBox, typename Iterator>
BBox get_parent_bbox(const Iterator begin, const Iterator end)
{
    BBox bbox;
    bbox.invalidate();

    for (Iterator i = begin; i != end; ++i)
        bbox.insert(i->get_parent_bbox());

    return bbox;
}

template <typename BBox, typename Iterator>
BBox compute_parent_bbox(const Iterator begin, const Iterator end)
{
    BBox bbox;
    bbox.invalidate();

    for (Iterator i = begin; i != end; ++i)
        bbox.insert(i->compute_parent_bbox());

    return bbox;
}

template <typename BBox, typename Iterator>
BBox compute_union(const Iterator begin, const Iterator end)
{
    assert(begin != end);

    Iterator i = begin;
    BBox result = *i++;

    while (i != end)
        result.insert(*i++);

    return result;
}

template <typename BBox, typename Iterator>
BBox interpolate(const Iterator begin, const Iterator end, const double time)
{
    assert(begin != end);
    assert(time >= 0.0 && time < 1.0);

    typedef typename BBox::ValueType ValueType;

    const BBox* first = &(*begin);
    const size_t bbox_count = end - begin;
    const size_t motion_segment_count = bbox_count - 1;
    const size_t prev_index = foundation::truncate<size_t>(time * motion_segment_count);
    const ValueType k = static_cast<ValueType>(time * motion_segment_count - prev_index);

    return foundation::lerp(first[prev_index], first[prev_index + 1], k);
}

}       // namespace renderer

#endif  // !APPLESEED_RENDERER_UTILITY_BBOX_H

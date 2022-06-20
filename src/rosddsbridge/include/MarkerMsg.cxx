// Copyright 2016 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*!
 * @file MarkerMsg.cpp
 * This source file contains the definition of the described types in the IDL file.
 *
 * This file was generated by the tool gen.
 */

#ifdef _WIN32
// Remove linker warning LNK4221 on Visual Studio
namespace {
char dummy;
}  // namespace
#endif  // _WIN32

#include "MarkerMsg.h"
#include <fastcdr/Cdr.h>

#include <fastcdr/exceptions/BadParamException.h>
using namespace eprosima::fastcdr::exception;

#include <utility>

MarkerMsg::MarkerMsg()
{
    // m_seq com.eprosima.idl.parser.typecode.PrimitiveTypeCode@17d0685f
    m_seq = 0;
    // m_secs com.eprosima.idl.parser.typecode.PrimitiveTypeCode@3891771e
    m_secs = 0;
    // m_nsecs com.eprosima.idl.parser.typecode.PrimitiveTypeCode@78ac1102
    m_nsecs = 0;
    // m_frame_id com.eprosima.idl.parser.typecode.StringTypeCode@2de8284b
    m_frame_id ="";
    // m_ns com.eprosima.idl.parser.typecode.StringTypeCode@396e2f39
    m_ns ="";
    // m_id com.eprosima.idl.parser.typecode.PrimitiveTypeCode@a74868d
    m_id = 0;
    // m_type com.eprosima.idl.parser.typecode.PrimitiveTypeCode@12c8a2c0
    m_type = 0;
    // m_action com.eprosima.idl.parser.typecode.PrimitiveTypeCode@7e0e6aa2
    m_action = 0;
    // m_position_x com.eprosima.idl.parser.typecode.PrimitiveTypeCode@365185bd
    m_position_x = 0.0;
    // m_position_y com.eprosima.idl.parser.typecode.PrimitiveTypeCode@18bf3d14
    m_position_y = 0.0;
    // m_position_z com.eprosima.idl.parser.typecode.PrimitiveTypeCode@4fb64261
    m_position_z = 0.0;
    // m_orientation_x com.eprosima.idl.parser.typecode.PrimitiveTypeCode@42607a4f
    m_orientation_x = 0.0;
    // m_orientation_y com.eprosima.idl.parser.typecode.PrimitiveTypeCode@782663d3
    m_orientation_y = 0.0;
    // m_orientation_z com.eprosima.idl.parser.typecode.PrimitiveTypeCode@1990a65e
    m_orientation_z = 0.0;
    // m_orientation_w com.eprosima.idl.parser.typecode.PrimitiveTypeCode@64485a47
    m_orientation_w = 0.0;
    // m_scale_x com.eprosima.idl.parser.typecode.PrimitiveTypeCode@7276c8cd
    m_scale_x = 0.0;
    // m_scale_y com.eprosima.idl.parser.typecode.PrimitiveTypeCode@150c158
    m_scale_y = 0.0;
    // m_scale_z com.eprosima.idl.parser.typecode.PrimitiveTypeCode@4524411f
    m_scale_z = 0.0;
    // m_color_r com.eprosima.idl.parser.typecode.PrimitiveTypeCode@43a0cee9
    m_color_r = 0.0;
    // m_color_g com.eprosima.idl.parser.typecode.PrimitiveTypeCode@eb21112
    m_color_g = 0.0;
    // m_color_b com.eprosima.idl.parser.typecode.PrimitiveTypeCode@2eda0940
    m_color_b = 0.0;
    // m_color_a com.eprosima.idl.parser.typecode.PrimitiveTypeCode@3578436e
    m_color_a = 0.0;

}

MarkerMsg::~MarkerMsg()
{






















}

MarkerMsg::MarkerMsg(
        const MarkerMsg& x)
{
    m_seq = x.m_seq;
    m_secs = x.m_secs;
    m_nsecs = x.m_nsecs;
    m_frame_id = x.m_frame_id;
    m_ns = x.m_ns;
    m_id = x.m_id;
    m_type = x.m_type;
    m_action = x.m_action;
    m_position_x = x.m_position_x;
    m_position_y = x.m_position_y;
    m_position_z = x.m_position_z;
    m_orientation_x = x.m_orientation_x;
    m_orientation_y = x.m_orientation_y;
    m_orientation_z = x.m_orientation_z;
    m_orientation_w = x.m_orientation_w;
    m_scale_x = x.m_scale_x;
    m_scale_y = x.m_scale_y;
    m_scale_z = x.m_scale_z;
    m_color_r = x.m_color_r;
    m_color_g = x.m_color_g;
    m_color_b = x.m_color_b;
    m_color_a = x.m_color_a;
}

MarkerMsg::MarkerMsg(
        MarkerMsg&& x)
{
    m_seq = x.m_seq;
    m_secs = x.m_secs;
    m_nsecs = x.m_nsecs;
    m_frame_id = std::move(x.m_frame_id);
    m_ns = std::move(x.m_ns);
    m_id = x.m_id;
    m_type = x.m_type;
    m_action = x.m_action;
    m_position_x = x.m_position_x;
    m_position_y = x.m_position_y;
    m_position_z = x.m_position_z;
    m_orientation_x = x.m_orientation_x;
    m_orientation_y = x.m_orientation_y;
    m_orientation_z = x.m_orientation_z;
    m_orientation_w = x.m_orientation_w;
    m_scale_x = x.m_scale_x;
    m_scale_y = x.m_scale_y;
    m_scale_z = x.m_scale_z;
    m_color_r = x.m_color_r;
    m_color_g = x.m_color_g;
    m_color_b = x.m_color_b;
    m_color_a = x.m_color_a;
}

MarkerMsg& MarkerMsg::operator =(
        const MarkerMsg& x)
{

    m_seq = x.m_seq;
    m_secs = x.m_secs;
    m_nsecs = x.m_nsecs;
    m_frame_id = x.m_frame_id;
    m_ns = x.m_ns;
    m_id = x.m_id;
    m_type = x.m_type;
    m_action = x.m_action;
    m_position_x = x.m_position_x;
    m_position_y = x.m_position_y;
    m_position_z = x.m_position_z;
    m_orientation_x = x.m_orientation_x;
    m_orientation_y = x.m_orientation_y;
    m_orientation_z = x.m_orientation_z;
    m_orientation_w = x.m_orientation_w;
    m_scale_x = x.m_scale_x;
    m_scale_y = x.m_scale_y;
    m_scale_z = x.m_scale_z;
    m_color_r = x.m_color_r;
    m_color_g = x.m_color_g;
    m_color_b = x.m_color_b;
    m_color_a = x.m_color_a;

    return *this;
}

MarkerMsg& MarkerMsg::operator =(
        MarkerMsg&& x)
{

    m_seq = x.m_seq;
    m_secs = x.m_secs;
    m_nsecs = x.m_nsecs;
    m_frame_id = std::move(x.m_frame_id);
    m_ns = std::move(x.m_ns);
    m_id = x.m_id;
    m_type = x.m_type;
    m_action = x.m_action;
    m_position_x = x.m_position_x;
    m_position_y = x.m_position_y;
    m_position_z = x.m_position_z;
    m_orientation_x = x.m_orientation_x;
    m_orientation_y = x.m_orientation_y;
    m_orientation_z = x.m_orientation_z;
    m_orientation_w = x.m_orientation_w;
    m_scale_x = x.m_scale_x;
    m_scale_y = x.m_scale_y;
    m_scale_z = x.m_scale_z;
    m_color_r = x.m_color_r;
    m_color_g = x.m_color_g;
    m_color_b = x.m_color_b;
    m_color_a = x.m_color_a;

    return *this;
}

size_t MarkerMsg::getMaxCdrSerializedSize(
        size_t current_alignment)
{
    size_t initial_alignment = current_alignment;


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4) + 255 + 1;

    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4) + 255 + 1;

    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);



    return current_alignment - initial_alignment;
}

size_t MarkerMsg::getCdrSerializedSize(
        const MarkerMsg& data,
        size_t current_alignment)
{
    (void)data;
    size_t initial_alignment = current_alignment;


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4) + data.frame_id().size() + 1;

    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4) + data.ns().size() + 1;

    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);



    return current_alignment - initial_alignment;
}

void MarkerMsg::serialize(
        eprosima::fastcdr::Cdr& scdr) const
{

    scdr << m_seq;
    scdr << m_secs;
    scdr << m_nsecs;
    scdr << m_frame_id;
    scdr << m_ns;
    scdr << m_id;
    scdr << m_type;
    scdr << m_action;
    scdr << m_position_x;
    scdr << m_position_y;
    scdr << m_position_z;
    scdr << m_orientation_x;
    scdr << m_orientation_y;
    scdr << m_orientation_z;
    scdr << m_orientation_w;
    scdr << m_scale_x;
    scdr << m_scale_y;
    scdr << m_scale_z;
    scdr << m_color_r;
    scdr << m_color_g;
    scdr << m_color_b;
    scdr << m_color_a;

}

void MarkerMsg::deserialize(
        eprosima::fastcdr::Cdr& dcdr)
{

    dcdr >> m_seq;
    dcdr >> m_secs;
    dcdr >> m_nsecs;
    dcdr >> m_frame_id;
    dcdr >> m_ns;
    dcdr >> m_id;
    dcdr >> m_type;
    dcdr >> m_action;
    dcdr >> m_position_x;
    dcdr >> m_position_y;
    dcdr >> m_position_z;
    dcdr >> m_orientation_x;
    dcdr >> m_orientation_y;
    dcdr >> m_orientation_z;
    dcdr >> m_orientation_w;
    dcdr >> m_scale_x;
    dcdr >> m_scale_y;
    dcdr >> m_scale_z;
    dcdr >> m_color_r;
    dcdr >> m_color_g;
    dcdr >> m_color_b;
    dcdr >> m_color_a;
}

/*!
 * @brief This function sets a value in member seq
 * @param _seq New value for member seq
 */
void MarkerMsg::seq(
        uint32_t _seq)
{
    m_seq = _seq;
}

/*!
 * @brief This function returns the value of member seq
 * @return Value of member seq
 */
uint32_t MarkerMsg::seq() const
{
    return m_seq;
}

/*!
 * @brief This function returns a reference to member seq
 * @return Reference to member seq
 */
uint32_t& MarkerMsg::seq()
{
    return m_seq;
}

/*!
 * @brief This function sets a value in member secs
 * @param _secs New value for member secs
 */
void MarkerMsg::secs(
        uint32_t _secs)
{
    m_secs = _secs;
}

/*!
 * @brief This function returns the value of member secs
 * @return Value of member secs
 */
uint32_t MarkerMsg::secs() const
{
    return m_secs;
}

/*!
 * @brief This function returns a reference to member secs
 * @return Reference to member secs
 */
uint32_t& MarkerMsg::secs()
{
    return m_secs;
}

/*!
 * @brief This function sets a value in member nsecs
 * @param _nsecs New value for member nsecs
 */
void MarkerMsg::nsecs(
        uint32_t _nsecs)
{
    m_nsecs = _nsecs;
}

/*!
 * @brief This function returns the value of member nsecs
 * @return Value of member nsecs
 */
uint32_t MarkerMsg::nsecs() const
{
    return m_nsecs;
}

/*!
 * @brief This function returns a reference to member nsecs
 * @return Reference to member nsecs
 */
uint32_t& MarkerMsg::nsecs()
{
    return m_nsecs;
}

/*!
 * @brief This function copies the value in member frame_id
 * @param _frame_id New value to be copied in member frame_id
 */
void MarkerMsg::frame_id(
        const std::string& _frame_id)
{
    m_frame_id = _frame_id;
}

/*!
 * @brief This function moves the value in member frame_id
 * @param _frame_id New value to be moved in member frame_id
 */
void MarkerMsg::frame_id(
        std::string&& _frame_id)
{
    m_frame_id = std::move(_frame_id);
}

/*!
 * @brief This function returns a constant reference to member frame_id
 * @return Constant reference to member frame_id
 */
const std::string& MarkerMsg::frame_id() const
{
    return m_frame_id;
}

/*!
 * @brief This function returns a reference to member frame_id
 * @return Reference to member frame_id
 */
std::string& MarkerMsg::frame_id()
{
    return m_frame_id;
}
/*!
 * @brief This function copies the value in member ns
 * @param _ns New value to be copied in member ns
 */
void MarkerMsg::ns(
        const std::string& _ns)
{
    m_ns = _ns;
}

/*!
 * @brief This function moves the value in member ns
 * @param _ns New value to be moved in member ns
 */
void MarkerMsg::ns(
        std::string&& _ns)
{
    m_ns = std::move(_ns);
}

/*!
 * @brief This function returns a constant reference to member ns
 * @return Constant reference to member ns
 */
const std::string& MarkerMsg::ns() const
{
    return m_ns;
}

/*!
 * @brief This function returns a reference to member ns
 * @return Reference to member ns
 */
std::string& MarkerMsg::ns()
{
    return m_ns;
}
/*!
 * @brief This function sets a value in member id
 * @param _id New value for member id
 */
void MarkerMsg::id(
        int32_t _id)
{
    m_id = _id;
}

/*!
 * @brief This function returns the value of member id
 * @return Value of member id
 */
int32_t MarkerMsg::id() const
{
    return m_id;
}

/*!
 * @brief This function returns a reference to member id
 * @return Reference to member id
 */
int32_t& MarkerMsg::id()
{
    return m_id;
}

/*!
 * @brief This function sets a value in member type
 * @param _type New value for member type
 */
void MarkerMsg::type(
        int32_t _type)
{
    m_type = _type;
}

/*!
 * @brief This function returns the value of member type
 * @return Value of member type
 */
int32_t MarkerMsg::type() const
{
    return m_type;
}

/*!
 * @brief This function returns a reference to member type
 * @return Reference to member type
 */
int32_t& MarkerMsg::type()
{
    return m_type;
}

/*!
 * @brief This function sets a value in member action
 * @param _action New value for member action
 */
void MarkerMsg::action(
        int32_t _action)
{
    m_action = _action;
}

/*!
 * @brief This function returns the value of member action
 * @return Value of member action
 */
int32_t MarkerMsg::action() const
{
    return m_action;
}

/*!
 * @brief This function returns a reference to member action
 * @return Reference to member action
 */
int32_t& MarkerMsg::action()
{
    return m_action;
}

/*!
 * @brief This function sets a value in member position_x
 * @param _position_x New value for member position_x
 */
void MarkerMsg::position_x(
        float _position_x)
{
    m_position_x = _position_x;
}

/*!
 * @brief This function returns the value of member position_x
 * @return Value of member position_x
 */
float MarkerMsg::position_x() const
{
    return m_position_x;
}

/*!
 * @brief This function returns a reference to member position_x
 * @return Reference to member position_x
 */
float& MarkerMsg::position_x()
{
    return m_position_x;
}

/*!
 * @brief This function sets a value in member position_y
 * @param _position_y New value for member position_y
 */
void MarkerMsg::position_y(
        float _position_y)
{
    m_position_y = _position_y;
}

/*!
 * @brief This function returns the value of member position_y
 * @return Value of member position_y
 */
float MarkerMsg::position_y() const
{
    return m_position_y;
}

/*!
 * @brief This function returns a reference to member position_y
 * @return Reference to member position_y
 */
float& MarkerMsg::position_y()
{
    return m_position_y;
}

/*!
 * @brief This function sets a value in member position_z
 * @param _position_z New value for member position_z
 */
void MarkerMsg::position_z(
        float _position_z)
{
    m_position_z = _position_z;
}

/*!
 * @brief This function returns the value of member position_z
 * @return Value of member position_z
 */
float MarkerMsg::position_z() const
{
    return m_position_z;
}

/*!
 * @brief This function returns a reference to member position_z
 * @return Reference to member position_z
 */
float& MarkerMsg::position_z()
{
    return m_position_z;
}

/*!
 * @brief This function sets a value in member orientation_x
 * @param _orientation_x New value for member orientation_x
 */
void MarkerMsg::orientation_x(
        float _orientation_x)
{
    m_orientation_x = _orientation_x;
}

/*!
 * @brief This function returns the value of member orientation_x
 * @return Value of member orientation_x
 */
float MarkerMsg::orientation_x() const
{
    return m_orientation_x;
}

/*!
 * @brief This function returns a reference to member orientation_x
 * @return Reference to member orientation_x
 */
float& MarkerMsg::orientation_x()
{
    return m_orientation_x;
}

/*!
 * @brief This function sets a value in member orientation_y
 * @param _orientation_y New value for member orientation_y
 */
void MarkerMsg::orientation_y(
        float _orientation_y)
{
    m_orientation_y = _orientation_y;
}

/*!
 * @brief This function returns the value of member orientation_y
 * @return Value of member orientation_y
 */
float MarkerMsg::orientation_y() const
{
    return m_orientation_y;
}

/*!
 * @brief This function returns a reference to member orientation_y
 * @return Reference to member orientation_y
 */
float& MarkerMsg::orientation_y()
{
    return m_orientation_y;
}

/*!
 * @brief This function sets a value in member orientation_z
 * @param _orientation_z New value for member orientation_z
 */
void MarkerMsg::orientation_z(
        float _orientation_z)
{
    m_orientation_z = _orientation_z;
}

/*!
 * @brief This function returns the value of member orientation_z
 * @return Value of member orientation_z
 */
float MarkerMsg::orientation_z() const
{
    return m_orientation_z;
}

/*!
 * @brief This function returns a reference to member orientation_z
 * @return Reference to member orientation_z
 */
float& MarkerMsg::orientation_z()
{
    return m_orientation_z;
}

/*!
 * @brief This function sets a value in member orientation_w
 * @param _orientation_w New value for member orientation_w
 */
void MarkerMsg::orientation_w(
        float _orientation_w)
{
    m_orientation_w = _orientation_w;
}

/*!
 * @brief This function returns the value of member orientation_w
 * @return Value of member orientation_w
 */
float MarkerMsg::orientation_w() const
{
    return m_orientation_w;
}

/*!
 * @brief This function returns a reference to member orientation_w
 * @return Reference to member orientation_w
 */
float& MarkerMsg::orientation_w()
{
    return m_orientation_w;
}

/*!
 * @brief This function sets a value in member scale_x
 * @param _scale_x New value for member scale_x
 */
void MarkerMsg::scale_x(
        float _scale_x)
{
    m_scale_x = _scale_x;
}

/*!
 * @brief This function returns the value of member scale_x
 * @return Value of member scale_x
 */
float MarkerMsg::scale_x() const
{
    return m_scale_x;
}

/*!
 * @brief This function returns a reference to member scale_x
 * @return Reference to member scale_x
 */
float& MarkerMsg::scale_x()
{
    return m_scale_x;
}

/*!
 * @brief This function sets a value in member scale_y
 * @param _scale_y New value for member scale_y
 */
void MarkerMsg::scale_y(
        float _scale_y)
{
    m_scale_y = _scale_y;
}

/*!
 * @brief This function returns the value of member scale_y
 * @return Value of member scale_y
 */
float MarkerMsg::scale_y() const
{
    return m_scale_y;
}

/*!
 * @brief This function returns a reference to member scale_y
 * @return Reference to member scale_y
 */
float& MarkerMsg::scale_y()
{
    return m_scale_y;
}

/*!
 * @brief This function sets a value in member scale_z
 * @param _scale_z New value for member scale_z
 */
void MarkerMsg::scale_z(
        float _scale_z)
{
    m_scale_z = _scale_z;
}

/*!
 * @brief This function returns the value of member scale_z
 * @return Value of member scale_z
 */
float MarkerMsg::scale_z() const
{
    return m_scale_z;
}

/*!
 * @brief This function returns a reference to member scale_z
 * @return Reference to member scale_z
 */
float& MarkerMsg::scale_z()
{
    return m_scale_z;
}

/*!
 * @brief This function sets a value in member color_r
 * @param _color_r New value for member color_r
 */
void MarkerMsg::color_r(
        float _color_r)
{
    m_color_r = _color_r;
}

/*!
 * @brief This function returns the value of member color_r
 * @return Value of member color_r
 */
float MarkerMsg::color_r() const
{
    return m_color_r;
}

/*!
 * @brief This function returns a reference to member color_r
 * @return Reference to member color_r
 */
float& MarkerMsg::color_r()
{
    return m_color_r;
}

/*!
 * @brief This function sets a value in member color_g
 * @param _color_g New value for member color_g
 */
void MarkerMsg::color_g(
        float _color_g)
{
    m_color_g = _color_g;
}

/*!
 * @brief This function returns the value of member color_g
 * @return Value of member color_g
 */
float MarkerMsg::color_g() const
{
    return m_color_g;
}

/*!
 * @brief This function returns a reference to member color_g
 * @return Reference to member color_g
 */
float& MarkerMsg::color_g()
{
    return m_color_g;
}

/*!
 * @brief This function sets a value in member color_b
 * @param _color_b New value for member color_b
 */
void MarkerMsg::color_b(
        float _color_b)
{
    m_color_b = _color_b;
}

/*!
 * @brief This function returns the value of member color_b
 * @return Value of member color_b
 */
float MarkerMsg::color_b() const
{
    return m_color_b;
}

/*!
 * @brief This function returns a reference to member color_b
 * @return Reference to member color_b
 */
float& MarkerMsg::color_b()
{
    return m_color_b;
}

/*!
 * @brief This function sets a value in member color_a
 * @param _color_a New value for member color_a
 */
void MarkerMsg::color_a(
        float _color_a)
{
    m_color_a = _color_a;
}

/*!
 * @brief This function returns the value of member color_a
 * @return Value of member color_a
 */
float MarkerMsg::color_a() const
{
    return m_color_a;
}

/*!
 * @brief This function returns a reference to member color_a
 * @return Reference to member color_a
 */
float& MarkerMsg::color_a()
{
    return m_color_a;
}


size_t MarkerMsg::getKeyMaxCdrSerializedSize(
        size_t current_alignment)
{
    size_t current_align = current_alignment;

























    return current_align;
}

bool MarkerMsg::isKeyDefined()
{
    return false;
}

void MarkerMsg::serializeKey(
        eprosima::fastcdr::Cdr& scdr) const
{
    (void) scdr;
                          
}

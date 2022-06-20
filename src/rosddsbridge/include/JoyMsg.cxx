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
 * @file JoyMsg.cpp
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

#include "JoyMsg.h"
#include <fastcdr/Cdr.h>

#include <fastcdr/exceptions/BadParamException.h>
using namespace eprosima::fastcdr::exception;

#include <utility>

JoyMsg::JoyMsg()
{
    // m_seq com.eprosima.idl.parser.typecode.PrimitiveTypeCode@5204062d
    m_seq = 0;
    // m_secs com.eprosima.idl.parser.typecode.PrimitiveTypeCode@4fcd19b3
    m_secs = 0;
    // m_nsecs com.eprosima.idl.parser.typecode.PrimitiveTypeCode@376b4233
    m_nsecs = 0;
    // m_frame_id com.eprosima.idl.parser.typecode.StringTypeCode@2fd66ad3
    m_frame_id ="";
    // m_axes com.eprosima.idl.parser.typecode.SequenceTypeCode@5d11346a

    // m_buttons com.eprosima.idl.parser.typecode.SequenceTypeCode@7a36aefa


}

JoyMsg::~JoyMsg()
{






}

JoyMsg::JoyMsg(
        const JoyMsg& x)
{
    m_seq = x.m_seq;
    m_secs = x.m_secs;
    m_nsecs = x.m_nsecs;
    m_frame_id = x.m_frame_id;
    m_axes = x.m_axes;
    m_buttons = x.m_buttons;
}

JoyMsg::JoyMsg(
        JoyMsg&& x)
{
    m_seq = x.m_seq;
    m_secs = x.m_secs;
    m_nsecs = x.m_nsecs;
    m_frame_id = std::move(x.m_frame_id);
    m_axes = std::move(x.m_axes);
    m_buttons = std::move(x.m_buttons);
}

JoyMsg& JoyMsg::operator =(
        const JoyMsg& x)
{

    m_seq = x.m_seq;
    m_secs = x.m_secs;
    m_nsecs = x.m_nsecs;
    m_frame_id = x.m_frame_id;
    m_axes = x.m_axes;
    m_buttons = x.m_buttons;

    return *this;
}

JoyMsg& JoyMsg::operator =(
        JoyMsg&& x)
{

    m_seq = x.m_seq;
    m_secs = x.m_secs;
    m_nsecs = x.m_nsecs;
    m_frame_id = std::move(x.m_frame_id);
    m_axes = std::move(x.m_axes);
    m_buttons = std::move(x.m_buttons);

    return *this;
}

size_t JoyMsg::getMaxCdrSerializedSize(
        size_t current_alignment)
{
    size_t initial_alignment = current_alignment;


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4) + 255 + 1;

    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);

    current_alignment += (100 * 4) + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);



    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);

    current_alignment += (100 * 4) + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);




    return current_alignment - initial_alignment;
}

size_t JoyMsg::getCdrSerializedSize(
        const JoyMsg& data,
        size_t current_alignment)
{
    (void)data;
    size_t initial_alignment = current_alignment;


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4) + data.frame_id().size() + 1;

    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);

    if (data.axes().size() > 0)
    {
        current_alignment += (data.axes().size() * 4) + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);
    }



    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);

    if (data.buttons().size() > 0)
    {
        current_alignment += (data.buttons().size() * 4) + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);
    }




    return current_alignment - initial_alignment;
}

void JoyMsg::serialize(
        eprosima::fastcdr::Cdr& scdr) const
{

    scdr << m_seq;
    scdr << m_secs;
    scdr << m_nsecs;
    scdr << m_frame_id;
    scdr << m_axes;
    scdr << m_buttons;

}

void JoyMsg::deserialize(
        eprosima::fastcdr::Cdr& dcdr)
{

    dcdr >> m_seq;
    dcdr >> m_secs;
    dcdr >> m_nsecs;
    dcdr >> m_frame_id;
    dcdr >> m_axes;
    dcdr >> m_buttons;
}

/*!
 * @brief This function sets a value in member seq
 * @param _seq New value for member seq
 */
void JoyMsg::seq(
        uint32_t _seq)
{
    m_seq = _seq;
}

/*!
 * @brief This function returns the value of member seq
 * @return Value of member seq
 */
uint32_t JoyMsg::seq() const
{
    return m_seq;
}

/*!
 * @brief This function returns a reference to member seq
 * @return Reference to member seq
 */
uint32_t& JoyMsg::seq()
{
    return m_seq;
}

/*!
 * @brief This function sets a value in member secs
 * @param _secs New value for member secs
 */
void JoyMsg::secs(
        uint32_t _secs)
{
    m_secs = _secs;
}

/*!
 * @brief This function returns the value of member secs
 * @return Value of member secs
 */
uint32_t JoyMsg::secs() const
{
    return m_secs;
}

/*!
 * @brief This function returns a reference to member secs
 * @return Reference to member secs
 */
uint32_t& JoyMsg::secs()
{
    return m_secs;
}

/*!
 * @brief This function sets a value in member nsecs
 * @param _nsecs New value for member nsecs
 */
void JoyMsg::nsecs(
        uint32_t _nsecs)
{
    m_nsecs = _nsecs;
}

/*!
 * @brief This function returns the value of member nsecs
 * @return Value of member nsecs
 */
uint32_t JoyMsg::nsecs() const
{
    return m_nsecs;
}

/*!
 * @brief This function returns a reference to member nsecs
 * @return Reference to member nsecs
 */
uint32_t& JoyMsg::nsecs()
{
    return m_nsecs;
}

/*!
 * @brief This function copies the value in member frame_id
 * @param _frame_id New value to be copied in member frame_id
 */
void JoyMsg::frame_id(
        const std::string& _frame_id)
{
    m_frame_id = _frame_id;
}

/*!
 * @brief This function moves the value in member frame_id
 * @param _frame_id New value to be moved in member frame_id
 */
void JoyMsg::frame_id(
        std::string&& _frame_id)
{
    m_frame_id = std::move(_frame_id);
}

/*!
 * @brief This function returns a constant reference to member frame_id
 * @return Constant reference to member frame_id
 */
const std::string& JoyMsg::frame_id() const
{
    return m_frame_id;
}

/*!
 * @brief This function returns a reference to member frame_id
 * @return Reference to member frame_id
 */
std::string& JoyMsg::frame_id()
{
    return m_frame_id;
}
/*!
 * @brief This function copies the value in member axes
 * @param _axes New value to be copied in member axes
 */
void JoyMsg::axes(
        const std::vector<float>& _axes)
{
    m_axes = _axes;
}

/*!
 * @brief This function moves the value in member axes
 * @param _axes New value to be moved in member axes
 */
void JoyMsg::axes(
        std::vector<float>&& _axes)
{
    m_axes = std::move(_axes);
}

/*!
 * @brief This function returns a constant reference to member axes
 * @return Constant reference to member axes
 */
const std::vector<float>& JoyMsg::axes() const
{
    return m_axes;
}

/*!
 * @brief This function returns a reference to member axes
 * @return Reference to member axes
 */
std::vector<float>& JoyMsg::axes()
{
    return m_axes;
}
/*!
 * @brief This function copies the value in member buttons
 * @param _buttons New value to be copied in member buttons
 */
void JoyMsg::buttons(
        const std::vector<int32_t>& _buttons)
{
    m_buttons = _buttons;
}

/*!
 * @brief This function moves the value in member buttons
 * @param _buttons New value to be moved in member buttons
 */
void JoyMsg::buttons(
        std::vector<int32_t>&& _buttons)
{
    m_buttons = std::move(_buttons);
}

/*!
 * @brief This function returns a constant reference to member buttons
 * @return Constant reference to member buttons
 */
const std::vector<int32_t>& JoyMsg::buttons() const
{
    return m_buttons;
}

/*!
 * @brief This function returns a reference to member buttons
 * @return Reference to member buttons
 */
std::vector<int32_t>& JoyMsg::buttons()
{
    return m_buttons;
}

size_t JoyMsg::getKeyMaxCdrSerializedSize(
        size_t current_alignment)
{
    size_t current_align = current_alignment;









    return current_align;
}

bool JoyMsg::isKeyDefined()
{
    return false;
}

void JoyMsg::serializeKey(
        eprosima::fastcdr::Cdr& scdr) const
{
    (void) scdr;
          
}
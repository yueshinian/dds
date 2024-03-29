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
 * @file PclMsg.h
 * This header file contains the declaration of the described types in the IDL file.
 *
 * This file was generated by the tool gen.
 */

#ifndef _PCLMSG_H_
#define _PCLMSG_H_


#include <stdint.h>
#include <array>
#include <string>
#include <vector>
#include <map>
#include <bitset>

#if defined(_WIN32)
#if defined(EPROSIMA_USER_DLL_EXPORT)
#define eProsima_user_DllExport __declspec( dllexport )
#else
#define eProsima_user_DllExport
#endif  // EPROSIMA_USER_DLL_EXPORT
#else
#define eProsima_user_DllExport
#endif  // _WIN32

#if defined(_WIN32)
#if defined(EPROSIMA_USER_DLL_EXPORT)
#if defined(PclMsg_SOURCE)
#define PclMsg_DllAPI __declspec( dllexport )
#else
#define PclMsg_DllAPI __declspec( dllimport )
#endif // PclMsg_SOURCE
#else
#define PclMsg_DllAPI
#endif  // EPROSIMA_USER_DLL_EXPORT
#else
#define PclMsg_DllAPI
#endif // _WIN32

namespace eprosima {
namespace fastcdr {
class Cdr;
} // namespace fastcdr
} // namespace eprosima


/*!
 * @brief This class represents the structure PclMsg defined by the user in the IDL file.
 * @ingroup PCLMSG
 */
class PclMsg
{
public:

    /*!
     * @brief Default constructor.
     */
    eProsima_user_DllExport PclMsg();

    /*!
     * @brief Default destructor.
     */
    eProsima_user_DllExport ~PclMsg();

    /*!
     * @brief Copy constructor.
     * @param x Reference to the object PclMsg that will be copied.
     */
    eProsima_user_DllExport PclMsg(
            const PclMsg& x);

    /*!
     * @brief Move constructor.
     * @param x Reference to the object PclMsg that will be copied.
     */
    eProsima_user_DllExport PclMsg(
            PclMsg&& x);

    /*!
     * @brief Copy assignment.
     * @param x Reference to the object PclMsg that will be copied.
     */
    eProsima_user_DllExport PclMsg& operator =(
            const PclMsg& x);

    /*!
     * @brief Move assignment.
     * @param x Reference to the object PclMsg that will be copied.
     */
    eProsima_user_DllExport PclMsg& operator =(
            PclMsg&& x);

    /*!
     * @brief This function sets a value in member seq
     * @param _seq New value for member seq
     */
    eProsima_user_DllExport void seq(
            uint32_t _seq);

    /*!
     * @brief This function returns the value of member seq
     * @return Value of member seq
     */
    eProsima_user_DllExport uint32_t seq() const;

    /*!
     * @brief This function returns a reference to member seq
     * @return Reference to member seq
     */
    eProsima_user_DllExport uint32_t& seq();

    /*!
     * @brief This function sets a value in member secs
     * @param _secs New value for member secs
     */
    eProsima_user_DllExport void secs(
            uint32_t _secs);

    /*!
     * @brief This function returns the value of member secs
     * @return Value of member secs
     */
    eProsima_user_DllExport uint32_t secs() const;

    /*!
     * @brief This function returns a reference to member secs
     * @return Reference to member secs
     */
    eProsima_user_DllExport uint32_t& secs();

    /*!
     * @brief This function sets a value in member nsecs
     * @param _nsecs New value for member nsecs
     */
    eProsima_user_DllExport void nsecs(
            uint32_t _nsecs);

    /*!
     * @brief This function returns the value of member nsecs
     * @return Value of member nsecs
     */
    eProsima_user_DllExport uint32_t nsecs() const;

    /*!
     * @brief This function returns a reference to member nsecs
     * @return Reference to member nsecs
     */
    eProsima_user_DllExport uint32_t& nsecs();

    /*!
     * @brief This function copies the value in member frame_id
     * @param _frame_id New value to be copied in member frame_id
     */
    eProsima_user_DllExport void frame_id(
            const std::string& _frame_id);

    /*!
     * @brief This function moves the value in member frame_id
     * @param _frame_id New value to be moved in member frame_id
     */
    eProsima_user_DllExport void frame_id(
            std::string&& _frame_id);

    /*!
     * @brief This function returns a constant reference to member frame_id
     * @return Constant reference to member frame_id
     */
    eProsima_user_DllExport const std::string& frame_id() const;

    /*!
     * @brief This function returns a reference to member frame_id
     * @return Reference to member frame_id
     */
    eProsima_user_DllExport std::string& frame_id();
    /*!
     * @brief This function sets a value in member height
     * @param _height New value for member height
     */
    eProsima_user_DllExport void height(
            uint32_t _height);

    /*!
     * @brief This function returns the value of member height
     * @return Value of member height
     */
    eProsima_user_DllExport uint32_t height() const;

    /*!
     * @brief This function returns a reference to member height
     * @return Reference to member height
     */
    eProsima_user_DllExport uint32_t& height();

    /*!
     * @brief This function sets a value in member width
     * @param _width New value for member width
     */
    eProsima_user_DllExport void width(
            uint32_t _width);

    /*!
     * @brief This function returns the value of member width
     * @return Value of member width
     */
    eProsima_user_DllExport uint32_t width() const;

    /*!
     * @brief This function returns a reference to member width
     * @return Reference to member width
     */
    eProsima_user_DllExport uint32_t& width();

    /*!
     * @brief This function copies the value in member PointFileds_name
     * @param _PointFileds_name New value to be copied in member PointFileds_name
     */
    eProsima_user_DllExport void PointFileds_name(
            const std::array<std::string, 4>& _PointFileds_name);

    /*!
     * @brief This function moves the value in member PointFileds_name
     * @param _PointFileds_name New value to be moved in member PointFileds_name
     */
    eProsima_user_DllExport void PointFileds_name(
            std::array<std::string, 4>&& _PointFileds_name);

    /*!
     * @brief This function returns a constant reference to member PointFileds_name
     * @return Constant reference to member PointFileds_name
     */
    eProsima_user_DllExport const std::array<std::string, 4>& PointFileds_name() const;

    /*!
     * @brief This function returns a reference to member PointFileds_name
     * @return Reference to member PointFileds_name
     */
    eProsima_user_DllExport std::array<std::string, 4>& PointFileds_name();
    /*!
     * @brief This function copies the value in member PointFileds_offset
     * @param _PointFileds_offset New value to be copied in member PointFileds_offset
     */
    eProsima_user_DllExport void PointFileds_offset(
            const std::array<uint32_t, 4>& _PointFileds_offset);

    /*!
     * @brief This function moves the value in member PointFileds_offset
     * @param _PointFileds_offset New value to be moved in member PointFileds_offset
     */
    eProsima_user_DllExport void PointFileds_offset(
            std::array<uint32_t, 4>&& _PointFileds_offset);

    /*!
     * @brief This function returns a constant reference to member PointFileds_offset
     * @return Constant reference to member PointFileds_offset
     */
    eProsima_user_DllExport const std::array<uint32_t, 4>& PointFileds_offset() const;

    /*!
     * @brief This function returns a reference to member PointFileds_offset
     * @return Reference to member PointFileds_offset
     */
    eProsima_user_DllExport std::array<uint32_t, 4>& PointFileds_offset();
    /*!
     * @brief This function copies the value in member PointFileds_datatype
     * @param _PointFileds_datatype New value to be copied in member PointFileds_datatype
     */
    eProsima_user_DllExport void PointFileds_datatype(
            const std::array<uint8_t, 4>& _PointFileds_datatype);

    /*!
     * @brief This function moves the value in member PointFileds_datatype
     * @param _PointFileds_datatype New value to be moved in member PointFileds_datatype
     */
    eProsima_user_DllExport void PointFileds_datatype(
            std::array<uint8_t, 4>&& _PointFileds_datatype);

    /*!
     * @brief This function returns a constant reference to member PointFileds_datatype
     * @return Constant reference to member PointFileds_datatype
     */
    eProsima_user_DllExport const std::array<uint8_t, 4>& PointFileds_datatype() const;

    /*!
     * @brief This function returns a reference to member PointFileds_datatype
     * @return Reference to member PointFileds_datatype
     */
    eProsima_user_DllExport std::array<uint8_t, 4>& PointFileds_datatype();
    /*!
     * @brief This function copies the value in member PointFileds_count
     * @param _PointFileds_count New value to be copied in member PointFileds_count
     */
    eProsima_user_DllExport void PointFileds_count(
            const std::array<uint32_t, 4>& _PointFileds_count);

    /*!
     * @brief This function moves the value in member PointFileds_count
     * @param _PointFileds_count New value to be moved in member PointFileds_count
     */
    eProsima_user_DllExport void PointFileds_count(
            std::array<uint32_t, 4>&& _PointFileds_count);

    /*!
     * @brief This function returns a constant reference to member PointFileds_count
     * @return Constant reference to member PointFileds_count
     */
    eProsima_user_DllExport const std::array<uint32_t, 4>& PointFileds_count() const;

    /*!
     * @brief This function returns a reference to member PointFileds_count
     * @return Reference to member PointFileds_count
     */
    eProsima_user_DllExport std::array<uint32_t, 4>& PointFileds_count();
    /*!
     * @brief This function sets a value in member is_bigendian
     * @param _is_bigendian New value for member is_bigendian
     */
    eProsima_user_DllExport void is_bigendian(
            bool _is_bigendian);

    /*!
     * @brief This function returns the value of member is_bigendian
     * @return Value of member is_bigendian
     */
    eProsima_user_DllExport bool is_bigendian() const;

    /*!
     * @brief This function returns a reference to member is_bigendian
     * @return Reference to member is_bigendian
     */
    eProsima_user_DllExport bool& is_bigendian();

    /*!
     * @brief This function sets a value in member point_step
     * @param _point_step New value for member point_step
     */
    eProsima_user_DllExport void point_step(
            uint32_t _point_step);

    /*!
     * @brief This function returns the value of member point_step
     * @return Value of member point_step
     */
    eProsima_user_DllExport uint32_t point_step() const;

    /*!
     * @brief This function returns a reference to member point_step
     * @return Reference to member point_step
     */
    eProsima_user_DllExport uint32_t& point_step();

    /*!
     * @brief This function sets a value in member row_step
     * @param _row_step New value for member row_step
     */
    eProsima_user_DllExport void row_step(
            uint32_t _row_step);

    /*!
     * @brief This function returns the value of member row_step
     * @return Value of member row_step
     */
    eProsima_user_DllExport uint32_t row_step() const;

    /*!
     * @brief This function returns a reference to member row_step
     * @return Reference to member row_step
     */
    eProsima_user_DllExport uint32_t& row_step();

    /*!
     * @brief This function sets a value in member datacount
     * @param _datacount New value for member datacount
     */
    eProsima_user_DllExport void datacount(
            uint32_t _datacount);

    /*!
     * @brief This function returns the value of member datacount
     * @return Value of member datacount
     */
    eProsima_user_DllExport uint32_t datacount() const;

    /*!
     * @brief This function returns a reference to member datacount
     * @return Reference to member datacount
     */
    eProsima_user_DllExport uint32_t& datacount();

    /*!
     * @brief This function copies the value in member data
     * @param _data New value to be copied in member data
     */
    eProsima_user_DllExport void data(
            const std::vector<char>& _data);

    /*!
     * @brief This function moves the value in member data
     * @param _data New value to be moved in member data
     */
    eProsima_user_DllExport void data(
            std::vector<char>&& _data);

    /*!
     * @brief This function returns a constant reference to member data
     * @return Constant reference to member data
     */
    eProsima_user_DllExport const std::vector<char>& data() const;

    /*!
     * @brief This function returns a reference to member data
     * @return Reference to member data
     */
    eProsima_user_DllExport std::vector<char>& data();
    /*!
     * @brief This function sets a value in member is_dense
     * @param _is_dense New value for member is_dense
     */
    eProsima_user_DllExport void is_dense(
            bool _is_dense);

    /*!
     * @brief This function returns the value of member is_dense
     * @return Value of member is_dense
     */
    eProsima_user_DllExport bool is_dense() const;

    /*!
     * @brief This function returns a reference to member is_dense
     * @return Reference to member is_dense
     */
    eProsima_user_DllExport bool& is_dense();


    /*!
     * @brief This function returns the maximum serialized size of an object
     * depending on the buffer alignment.
     * @param current_alignment Buffer alignment.
     * @return Maximum serialized size.
     */
    eProsima_user_DllExport static size_t getMaxCdrSerializedSize(
            size_t current_alignment = 0);

    /*!
     * @brief This function returns the serialized size of a data depending on the buffer alignment.
     * @param data Data which is calculated its serialized size.
     * @param current_alignment Buffer alignment.
     * @return Serialized size.
     */
    eProsima_user_DllExport static size_t getCdrSerializedSize(
            const PclMsg& data,
            size_t current_alignment = 0);


    /*!
     * @brief This function serializes an object using CDR serialization.
     * @param cdr CDR serialization object.
     */
    eProsima_user_DllExport void serialize(
            eprosima::fastcdr::Cdr& cdr) const;

    /*!
     * @brief This function deserializes an object using CDR serialization.
     * @param cdr CDR serialization object.
     */
    eProsima_user_DllExport void deserialize(
            eprosima::fastcdr::Cdr& cdr);



    /*!
     * @brief This function returns the maximum serialized size of the Key of an object
     * depending on the buffer alignment.
     * @param current_alignment Buffer alignment.
     * @return Maximum serialized size.
     */
    eProsima_user_DllExport static size_t getKeyMaxCdrSerializedSize(
            size_t current_alignment = 0);

    /*!
     * @brief This function tells you if the Key has been defined for this type
     */
    eProsima_user_DllExport static bool isKeyDefined();

    /*!
     * @brief This function serializes the key members of an object using CDR serialization.
     * @param cdr CDR serialization object.
     */
    eProsima_user_DllExport void serializeKey(
            eprosima::fastcdr::Cdr& cdr) const;

private:

    uint32_t m_seq;
    uint32_t m_secs;
    uint32_t m_nsecs;
    std::string m_frame_id;
    uint32_t m_height;
    uint32_t m_width;
    std::array<std::string, 4> m_PointFileds_name;
    std::array<uint32_t, 4> m_PointFileds_offset;
    std::array<uint8_t, 4> m_PointFileds_datatype;
    std::array<uint32_t, 4> m_PointFileds_count;
    bool m_is_bigendian;
    uint32_t m_point_step;
    uint32_t m_row_step;
    uint32_t m_datacount;
    std::vector<char> m_data;
    bool m_is_dense;
};

#endif // _PCLMSG_H_
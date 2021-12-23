/*
 * UAVCAN data structure definition for libuavcan.
 *
 * Autogenerated, do not edit.
 *
 * Source file: /home/dtpmh/PX4-Autopilot_1.11/src/drivers/uavcan/libuavcan/dsdl/uavcan/protocol/file/40.BeginFirmwareUpdate.uavcan
 */

#ifndef UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_HPP_INCLUDED
#define UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/node/global_data_type_registry.hpp>
#include <uavcan/marshal/types.hpp>

#include <uavcan/protocol/file/Path.hpp>

/******************************* Source text **********************************
#
# This service initiates firmware update on a remote node.
#
# The node that is being updated (slave) will retrieve the firmware image file 'image_file_remote_path' from the node
# 'source_node_id' using the file read service, then it will update the firmware and reboot.
#
# The slave can explicitly reject this request if it is not possible to update the firmware at the moment
# (e.g. if the node is busy).
#
# If the slave node accepts this request, the initiator will get a response immediately, before the update process
# actually begins.
#
# While the firmware is being updated, the slave should set its mode (uavcan.protocol.NodeStatus.mode) to
# MODE_SOFTWARE_UPDATE.
#

uint8 source_node_id         # If this field is zero, the caller's Node ID will be used instead.
Path image_file_remote_path

---

#
# Other error codes may be added in the future.
#
uint8 ERROR_OK               = 0
uint8 ERROR_INVALID_MODE     = 1    # Cannot perform the update in the current operating mode or state.
uint8 ERROR_IN_PROGRESS      = 2    # Firmware update is already in progress, and the slave doesn't want to restart.
uint8 ERROR_UNKNOWN          = 255
uint8 error

uint8[<128] optional_error_message   # Detailed description of the error.
******************************************************************************/

/********************* DSDL signature source definition ***********************
uavcan.protocol.file.BeginFirmwareUpdate
saturated uint8 source_node_id
uavcan.protocol.file.Path image_file_remote_path
---
saturated uint8 error
saturated uint8[<=127] optional_error_message
******************************************************************************/

#undef source_node_id
#undef image_file_remote_path
#undef error
#undef optional_error_message
#undef ERROR_OK
#undef ERROR_INVALID_MODE
#undef ERROR_IN_PROGRESS
#undef ERROR_UNKNOWN

namespace uavcan
{
namespace protocol
{
namespace file
{

struct UAVCAN_EXPORT BeginFirmwareUpdate_
{
    template <int _tmpl>
    struct Request_
    {
        typedef const Request_<_tmpl>& ParameterType;
        typedef Request_<_tmpl>& ReferenceType;

        struct ConstantTypes
        {
        };

        struct FieldTypes
        {
            typedef ::uavcan::IntegerSpec< 8, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > source_node_id;
            typedef ::uavcan::protocol::file::Path image_file_remote_path;
        };

        enum
        {
            MinBitLen
                = FieldTypes::source_node_id::MinBitLen
                + FieldTypes::image_file_remote_path::MinBitLen
        };

        enum
        {
            MaxBitLen
                = FieldTypes::source_node_id::MaxBitLen
                + FieldTypes::image_file_remote_path::MaxBitLen
        };

        // Constants

        // Fields
        typename ::uavcan::StorageType< typename FieldTypes::source_node_id >::Type source_node_id;
        typename ::uavcan::StorageType< typename FieldTypes::image_file_remote_path >::Type image_file_remote_path;

        Request_()
            : source_node_id()
            , image_file_remote_path()
        {
            ::uavcan::StaticAssert<_tmpl == 0>::check();  // Usage check

    #if UAVCAN_DEBUG
            /*
             * Cross-checking MaxBitLen provided by the DSDL compiler.
             * This check shall never be performed in user code because MaxBitLen value
             * actually depends on the nested types, thus it is not invariant.
             */
            ::uavcan::StaticAssert<1616 == MaxBitLen>::check();
    #endif
        }

        bool operator==(ParameterType rhs) const;
        bool operator!=(ParameterType rhs) const { return !operator==(rhs); }

        /**
         * This comparison is based on @ref uavcan::areClose(), which ensures proper comparison of
         * floating point fields at any depth.
         */
        bool isClose(ParameterType rhs) const;

        static int encode(ParameterType self, ::uavcan::ScalarCodec& codec,
                          ::uavcan::TailArrayOptimizationMode tao_mode = ::uavcan::TailArrayOptEnabled);

        static int decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
                          ::uavcan::TailArrayOptimizationMode tao_mode = ::uavcan::TailArrayOptEnabled);

    };

    template <int _tmpl>
    struct Response_
    {
        typedef const Response_<_tmpl>& ParameterType;
        typedef Response_<_tmpl>& ReferenceType;

        struct ConstantTypes
        {
            typedef ::uavcan::IntegerSpec< 8, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > ERROR_OK;
            typedef ::uavcan::IntegerSpec< 8, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > ERROR_INVALID_MODE;
            typedef ::uavcan::IntegerSpec< 8, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > ERROR_IN_PROGRESS;
            typedef ::uavcan::IntegerSpec< 8, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > ERROR_UNKNOWN;
        };

        struct FieldTypes
        {
            typedef ::uavcan::IntegerSpec< 8, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > error;
            typedef ::uavcan::Array< ::uavcan::IntegerSpec< 8, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate >, ::uavcan::ArrayModeDynamic, 127 > optional_error_message;
        };

        enum
        {
            MinBitLen
                = FieldTypes::error::MinBitLen
                + FieldTypes::optional_error_message::MinBitLen
        };

        enum
        {
            MaxBitLen
                = FieldTypes::error::MaxBitLen
                + FieldTypes::optional_error_message::MaxBitLen
        };

        // Constants
        static const typename ::uavcan::StorageType< typename ConstantTypes::ERROR_OK >::Type ERROR_OK; // 0
        static const typename ::uavcan::StorageType< typename ConstantTypes::ERROR_INVALID_MODE >::Type ERROR_INVALID_MODE; // 1
        static const typename ::uavcan::StorageType< typename ConstantTypes::ERROR_IN_PROGRESS >::Type ERROR_IN_PROGRESS; // 2
        static const typename ::uavcan::StorageType< typename ConstantTypes::ERROR_UNKNOWN >::Type ERROR_UNKNOWN; // 255

        // Fields
        typename ::uavcan::StorageType< typename FieldTypes::error >::Type error;
        typename ::uavcan::StorageType< typename FieldTypes::optional_error_message >::Type optional_error_message;

        Response_()
            : error()
            , optional_error_message()
        {
            ::uavcan::StaticAssert<_tmpl == 0>::check();  // Usage check

    #if UAVCAN_DEBUG
            /*
             * Cross-checking MaxBitLen provided by the DSDL compiler.
             * This check shall never be performed in user code because MaxBitLen value
             * actually depends on the nested types, thus it is not invariant.
             */
            ::uavcan::StaticAssert<1031 == MaxBitLen>::check();
    #endif
        }

        bool operator==(ParameterType rhs) const;
        bool operator!=(ParameterType rhs) const { return !operator==(rhs); }

        /**
         * This comparison is based on @ref uavcan::areClose(), which ensures proper comparison of
         * floating point fields at any depth.
         */
        bool isClose(ParameterType rhs) const;

        static int encode(ParameterType self, ::uavcan::ScalarCodec& codec,
                          ::uavcan::TailArrayOptimizationMode tao_mode = ::uavcan::TailArrayOptEnabled);

        static int decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
                          ::uavcan::TailArrayOptimizationMode tao_mode = ::uavcan::TailArrayOptEnabled);

    };

    typedef Request_<0> Request;
    typedef Response_<0> Response;

    /*
     * Static type info
     */
    enum { DataTypeKind = ::uavcan::DataTypeKindService };
    enum { DefaultDataTypeID = 40 };

    static const char* getDataTypeFullName()
    {
        return "uavcan.protocol.file.BeginFirmwareUpdate";
    }

    static void extendDataTypeSignature(::uavcan::DataTypeSignature& signature)
    {
        signature.extend(getDataTypeSignature());
    }

    static ::uavcan::DataTypeSignature getDataTypeSignature();

private:
    BeginFirmwareUpdate_(); // Don't create objects of this type. Use Request/Response instead.
};

/*
 * Out of line struct method definitions
 */

template <int _tmpl>
bool BeginFirmwareUpdate_::Request_<_tmpl>::operator==(ParameterType rhs) const
{
    return
        source_node_id == rhs.source_node_id &&
        image_file_remote_path == rhs.image_file_remote_path;
}

template <int _tmpl>
bool BeginFirmwareUpdate_::Request_<_tmpl>::isClose(ParameterType rhs) const
{
    return
        ::uavcan::areClose(source_node_id, rhs.source_node_id) &&
        ::uavcan::areClose(image_file_remote_path, rhs.image_file_remote_path);
}

template <int _tmpl>
int BeginFirmwareUpdate_::Request_<_tmpl>::encode(ParameterType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::source_node_id::encode(self.source_node_id, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::image_file_remote_path::encode(self.image_file_remote_path, codec,  tao_mode);
    return res;
}

template <int _tmpl>
int BeginFirmwareUpdate_::Request_<_tmpl>::decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::source_node_id::decode(self.source_node_id, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::image_file_remote_path::decode(self.image_file_remote_path, codec,  tao_mode);
    return res;
}

template <int _tmpl>
bool BeginFirmwareUpdate_::Response_<_tmpl>::operator==(ParameterType rhs) const
{
    return
        error == rhs.error &&
        optional_error_message == rhs.optional_error_message;
}

template <int _tmpl>
bool BeginFirmwareUpdate_::Response_<_tmpl>::isClose(ParameterType rhs) const
{
    return
        ::uavcan::areClose(error, rhs.error) &&
        ::uavcan::areClose(optional_error_message, rhs.optional_error_message);
}

template <int _tmpl>
int BeginFirmwareUpdate_::Response_<_tmpl>::encode(ParameterType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::error::encode(self.error, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::optional_error_message::encode(self.optional_error_message, codec,  tao_mode);
    return res;
}

template <int _tmpl>
int BeginFirmwareUpdate_::Response_<_tmpl>::decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::error::decode(self.error, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::optional_error_message::decode(self.optional_error_message, codec,  tao_mode);
    return res;
}

/*
 * Out of line type method definitions
 */
inline ::uavcan::DataTypeSignature BeginFirmwareUpdate_::getDataTypeSignature()
{
    ::uavcan::DataTypeSignature signature(0x36A8B8AA5453257BULL);

    Request::FieldTypes::source_node_id::extendDataTypeSignature(signature);
    Request::FieldTypes::image_file_remote_path::extendDataTypeSignature(signature);

    Response::FieldTypes::error::extendDataTypeSignature(signature);
    Response::FieldTypes::optional_error_message::extendDataTypeSignature(signature);

    return signature;
}

/*
 * Out of line constant definitions
 */

template <int _tmpl>
const typename ::uavcan::StorageType< typename BeginFirmwareUpdate_::Response_<_tmpl>::ConstantTypes::ERROR_OK >::Type
    BeginFirmwareUpdate_::Response_<_tmpl>::ERROR_OK = 0U; // 0

template <int _tmpl>
const typename ::uavcan::StorageType< typename BeginFirmwareUpdate_::Response_<_tmpl>::ConstantTypes::ERROR_INVALID_MODE >::Type
    BeginFirmwareUpdate_::Response_<_tmpl>::ERROR_INVALID_MODE = 1U; // 1

template <int _tmpl>
const typename ::uavcan::StorageType< typename BeginFirmwareUpdate_::Response_<_tmpl>::ConstantTypes::ERROR_IN_PROGRESS >::Type
    BeginFirmwareUpdate_::Response_<_tmpl>::ERROR_IN_PROGRESS = 2U; // 2

template <int _tmpl>
const typename ::uavcan::StorageType< typename BeginFirmwareUpdate_::Response_<_tmpl>::ConstantTypes::ERROR_UNKNOWN >::Type
    BeginFirmwareUpdate_::Response_<_tmpl>::ERROR_UNKNOWN = 255U; // 255

/*
 * Final typedef
 */
typedef BeginFirmwareUpdate_ BeginFirmwareUpdate;

namespace
{

const ::uavcan::DefaultDataTypeRegistrator< ::uavcan::protocol::file::BeginFirmwareUpdate > _uavcan_gdtr_registrator_BeginFirmwareUpdate;

}

} // Namespace file
} // Namespace protocol
} // Namespace uavcan

/*
 * YAML streamer specialization
 */
namespace uavcan
{

template <>
class UAVCAN_EXPORT YamlStreamer< ::uavcan::protocol::file::BeginFirmwareUpdate::Request >
{
public:
    template <typename Stream>
    static void stream(Stream& s, ::uavcan::protocol::file::BeginFirmwareUpdate::Request::ParameterType obj, const int level);
};

template <typename Stream>
void YamlStreamer< ::uavcan::protocol::file::BeginFirmwareUpdate::Request >::stream(Stream& s, ::uavcan::protocol::file::BeginFirmwareUpdate::Request::ParameterType obj, const int level)
{
    (void)s;
    (void)obj;
    (void)level;
    if (level > 0)
    {
        s << '\n';
        for (int pos = 0; pos < level; pos++)
        {
            s << "  ";
        }
    }
    s << "source_node_id: ";
    YamlStreamer< ::uavcan::protocol::file::BeginFirmwareUpdate::Request::FieldTypes::source_node_id >::stream(s, obj.source_node_id, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "image_file_remote_path: ";
    YamlStreamer< ::uavcan::protocol::file::BeginFirmwareUpdate::Request::FieldTypes::image_file_remote_path >::stream(s, obj.image_file_remote_path, level + 1);
}

template <>
class UAVCAN_EXPORT YamlStreamer< ::uavcan::protocol::file::BeginFirmwareUpdate::Response >
{
public:
    template <typename Stream>
    static void stream(Stream& s, ::uavcan::protocol::file::BeginFirmwareUpdate::Response::ParameterType obj, const int level);
};

template <typename Stream>
void YamlStreamer< ::uavcan::protocol::file::BeginFirmwareUpdate::Response >::stream(Stream& s, ::uavcan::protocol::file::BeginFirmwareUpdate::Response::ParameterType obj, const int level)
{
    (void)s;
    (void)obj;
    (void)level;
    if (level > 0)
    {
        s << '\n';
        for (int pos = 0; pos < level; pos++)
        {
            s << "  ";
        }
    }
    s << "error: ";
    YamlStreamer< ::uavcan::protocol::file::BeginFirmwareUpdate::Response::FieldTypes::error >::stream(s, obj.error, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "optional_error_message: ";
    YamlStreamer< ::uavcan::protocol::file::BeginFirmwareUpdate::Response::FieldTypes::optional_error_message >::stream(s, obj.optional_error_message, level + 1);
}

}

namespace uavcan
{
namespace protocol
{
namespace file
{

template <typename Stream>
inline Stream& operator<<(Stream& s, ::uavcan::protocol::file::BeginFirmwareUpdate::Request::ParameterType obj)
{
    ::uavcan::YamlStreamer< ::uavcan::protocol::file::BeginFirmwareUpdate::Request >::stream(s, obj, 0);
    return s;
}

template <typename Stream>
inline Stream& operator<<(Stream& s, ::uavcan::protocol::file::BeginFirmwareUpdate::Response::ParameterType obj)
{
    ::uavcan::YamlStreamer< ::uavcan::protocol::file::BeginFirmwareUpdate::Response >::stream(s, obj, 0);
    return s;
}

} // Namespace file
} // Namespace protocol
} // Namespace uavcan

#endif // UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_HPP_INCLUDED
/*
 * UAVCAN data structure definition for libuavcan.
 *
 * Autogenerated, do not edit.
 *
 * Source file: /home/dtpmh/PX4-Autopilot_1.11/src/drivers/uavcan/libuavcan/dsdl/uavcan/equipment/esc/1031.RPMCommand.uavcan
 */

#ifndef UAVCAN_EQUIPMENT_ESC_RPMCOMMAND_HPP_INCLUDED
#define UAVCAN_EQUIPMENT_ESC_RPMCOMMAND_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/node/global_data_type_registry.hpp>
#include <uavcan/marshal/types.hpp>

/******************************* Source text **********************************
#
# Simple RPM setpoint.
# The ESC should automatically clamp the setpoint according to the minimum and maximum supported RPM;
# for example, given a ESC that operates in the range 100 to 10000 RPM, a setpoint of 1 RPM will be clamped to 100 RPM.
# Negative values indicate reverse rotation.
#

int18[<=20] rpm
******************************************************************************/

/********************* DSDL signature source definition ***********************
uavcan.equipment.esc.RPMCommand
saturated int18[<=20] rpm
******************************************************************************/

#undef rpm

namespace uavcan
{
namespace equipment
{
namespace esc
{

template <int _tmpl>
struct UAVCAN_EXPORT RPMCommand_
{
    typedef const RPMCommand_<_tmpl>& ParameterType;
    typedef RPMCommand_<_tmpl>& ReferenceType;

    struct ConstantTypes
    {
    };

    struct FieldTypes
    {
        typedef ::uavcan::Array< ::uavcan::IntegerSpec< 18, ::uavcan::SignednessSigned, ::uavcan::CastModeSaturate >, ::uavcan::ArrayModeDynamic, 20 > rpm;
    };

    enum
    {
        MinBitLen
            = FieldTypes::rpm::MinBitLen
    };

    enum
    {
        MaxBitLen
            = FieldTypes::rpm::MaxBitLen
    };

    // Constants

    // Fields
    typename ::uavcan::StorageType< typename FieldTypes::rpm >::Type rpm;

    RPMCommand_()
        : rpm()
    {
        ::uavcan::StaticAssert<_tmpl == 0>::check();  // Usage check

#if UAVCAN_DEBUG
        /*
         * Cross-checking MaxBitLen provided by the DSDL compiler.
         * This check shall never be performed in user code because MaxBitLen value
         * actually depends on the nested types, thus it is not invariant.
         */
        ::uavcan::StaticAssert<365 == MaxBitLen>::check();
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

    /*
     * Static type info
     */
    enum { DataTypeKind = ::uavcan::DataTypeKindMessage };
    enum { DefaultDataTypeID = 1031 };

    static const char* getDataTypeFullName()
    {
        return "uavcan.equipment.esc.RPMCommand";
    }

    static void extendDataTypeSignature(::uavcan::DataTypeSignature& signature)
    {
        signature.extend(getDataTypeSignature());
    }

    static ::uavcan::DataTypeSignature getDataTypeSignature();

};

/*
 * Out of line struct method definitions
 */

template <int _tmpl>
bool RPMCommand_<_tmpl>::operator==(ParameterType rhs) const
{
    return
        rpm == rhs.rpm;
}

template <int _tmpl>
bool RPMCommand_<_tmpl>::isClose(ParameterType rhs) const
{
    return
        ::uavcan::areClose(rpm, rhs.rpm);
}

template <int _tmpl>
int RPMCommand_<_tmpl>::encode(ParameterType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::rpm::encode(self.rpm, codec,  tao_mode);
    return res;
}

template <int _tmpl>
int RPMCommand_<_tmpl>::decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::rpm::decode(self.rpm, codec,  tao_mode);
    return res;
}

/*
 * Out of line type method definitions
 */
template <int _tmpl>
::uavcan::DataTypeSignature RPMCommand_<_tmpl>::getDataTypeSignature()
{
    ::uavcan::DataTypeSignature signature(0xCE0F9F621CF7E70BULL);

    FieldTypes::rpm::extendDataTypeSignature(signature);

    return signature;
}

/*
 * Out of line constant definitions
 */

/*
 * Final typedef
 */
typedef RPMCommand_<0> RPMCommand;

namespace
{

const ::uavcan::DefaultDataTypeRegistrator< ::uavcan::equipment::esc::RPMCommand > _uavcan_gdtr_registrator_RPMCommand;

}

} // Namespace esc
} // Namespace equipment
} // Namespace uavcan

/*
 * YAML streamer specialization
 */
namespace uavcan
{

template <>
class UAVCAN_EXPORT YamlStreamer< ::uavcan::equipment::esc::RPMCommand >
{
public:
    template <typename Stream>
    static void stream(Stream& s, ::uavcan::equipment::esc::RPMCommand::ParameterType obj, const int level);
};

template <typename Stream>
void YamlStreamer< ::uavcan::equipment::esc::RPMCommand >::stream(Stream& s, ::uavcan::equipment::esc::RPMCommand::ParameterType obj, const int level)
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
    s << "rpm: ";
    YamlStreamer< ::uavcan::equipment::esc::RPMCommand::FieldTypes::rpm >::stream(s, obj.rpm, level + 1);
}

}

namespace uavcan
{
namespace equipment
{
namespace esc
{

template <typename Stream>
inline Stream& operator<<(Stream& s, ::uavcan::equipment::esc::RPMCommand::ParameterType obj)
{
    ::uavcan::YamlStreamer< ::uavcan::equipment::esc::RPMCommand >::stream(s, obj, 0);
    return s;
}

} // Namespace esc
} // Namespace equipment
} // Namespace uavcan

#endif // UAVCAN_EQUIPMENT_ESC_RPMCOMMAND_HPP_INCLUDED
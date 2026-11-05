#pragma once

#include <cstdint>

// SAShaderL public interface (features version 1 / interface v1.1).
enum eSaslUniformValueType : uint8_t {
    SASL_UNIFORM_INT = 0,
    SASL_UNIFORM_UINT,
    SASL_UNIFORM_FLOAT,
};

class ISASL {
public:
    virtual int GetFeaturesVersion() = 0;
    virtual int RegisterUniform(const char* name, eSaslUniformValueType type,
                                uint8_t valuesArraySize, bool alwaysUpdate = false,
                                void* pointer = nullptr);
    virtual void SetUniformInt(int id, int dataNum, int value);
    virtual void SetUniformUInt(int id, int dataNum, uint32_t value);
    virtual void SetUniformFloat(int id, int dataNum, float value);
    virtual void SetUniformPtr(int id, int dataNum, void* ptr);
    virtual void ForceUpdateData(int id);
};

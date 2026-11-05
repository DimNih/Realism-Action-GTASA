#pragma once

namespace ComplexShadowSystem {
    // Complex Shadow: realtime sun shadows for every nearby building / map object, with the outline
    // taken from the model's own DFF geometry, draped over the ground by the native shadow renderer.
    // World-light capture belongs to DynamicLightSystem.
    void Initialize();
}

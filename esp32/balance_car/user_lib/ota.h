#pragma once
namespace ota {
// Must run before motor initialization, including when initialization later fails.
void prepare_boot();
bool init(bool application_ready);
}

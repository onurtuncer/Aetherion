
#include "fmu4cpp/fmu_base.hpp"
#include <cstring>
#include "fmu4cpp/fmu_except.hpp"
#include "fmu4cpp/model_info.hpp"
#include "fmu4cpp/util.hpp"

#include "hash.hpp"

#include <algorithm>
#include <functional>
#include <sstream>
#include <utility>


using namespace fmu4cpp;

fmu_base::fmu_base(fmu_data data) : data_(std::move(data)) {

    register_real("time", &time_)
            .setCausality(causality_t::INDEPENDENT)
            .setVariability(variability_t::CONTINUOUS)
            .setDescription("Simulation time");
}


std::optional<IntVariable> fmu_base::get_int_variable(const std::string &name) const {
    for (const auto &v: integers_) {
        if (v.name() == name) return v;
    }
    return std::nullopt;
}

std::optional<RealVariable> fmu_base::get_real_variable(const std::string &name) const {
    for (const auto &v: reals_) {
        if (v.name() == name) return v;
    }
    return std::nullopt;
}

std::optional<BoolVariable> fmu_base::get_bool_variable(const std::string &name) const {
    for (const auto &v: booleans_) {
        if (v.name() == name) return v;
    }
    return std::nullopt;
}

std::optional<StringVariable> fmu_base::get_string_variable(const std::string &name) const {
    for (const auto &v: strings_) {
        if (v.name() == name) return v;
    }
    return std::nullopt;
}

std::optional<BinaryVariable> fmu_base::get_binary_variable(const std::string &name) const {
    for (const auto &v: binary_) {
        if (v.name() == name) return v;
    }
    return std::nullopt;
}

void fmu_base::enter_initialisation_mode(double start, std::optional<double> stop, std::optional<double> tolerance) {
    time_ = start;
    stop_ = stop;
    tolerance_ = tolerance;
    enter_initialisation_mode();
}

void fmu_base::enter_initialisation_mode() {}
void fmu_base::exit_initialisation_mode() {}

bool fmu_base::step(double currentTime, double dt) {

    if (stop_ && currentTime >= *stop_) {
        debugLog(fmiWarning, "Stop time reached");
        return false;
    }

    constexpr double TIME_TOLERANCE = 1e-9;
    if (std::abs(currentTime - time_) > TIME_TOLERANCE) {
        throw std::runtime_error("Current time does not match the internal time (within tolerance)");
    }

    if (do_step(dt)) {
        time_ += dt;

        return true;
    }

    return false;
}

void fmu_base::terminate() {}

void fmu_base::reset() {
    if (!state_ops_ || !get_state_ptr_) {
        throw fatal_error("Reset not implemented by FMU");
    }
    void *dst = get_state_ptr_(this);
    state_ops_->reset_inplace(dst);
}

void fmu_base::get_integer(const unsigned int vr[], size_t nvr, int value[]) const {
    for (unsigned i = 0; i < nvr; i++) {
        const auto ref = vr[i];
        const auto idx = vrToIntegerIndices_.at(ref);
        value[i] = integers_[idx].get();
    }
}

void fmu_base::get_real(const unsigned int vr[], size_t nvr, double value[]) const {
    for (unsigned i = 0; i < nvr; i++) {
        const auto ref = vr[i];
        const auto idx = vrToRealIndices_.at(ref);
        value[i] = reals_[idx].get();
    }
}

void fmu_base::get_boolean(const unsigned int vr[], size_t nvr, int value[]) const {
    for (unsigned i = 0; i < nvr; i++) {
        const auto ref = vr[i];
        const auto idx = vrToBooleanIndices_.at(ref);
        value[i] = static_cast<int>(booleans_[idx].get());
    }
}

void fmu_base::get_boolean(const unsigned int vr[], size_t nvr, bool value[]) const {
    for (unsigned i = 0; i < nvr; i++) {
        const auto ref = vr[i];
        const auto idx = vrToBooleanIndices_.at(ref);
        value[i] = booleans_[idx].get();
    }
}

void fmu_base::get_string(const unsigned int vr[], size_t nvr, const char *value[]) {
    stringBuffer_.clear();
    for (unsigned i = 0; i < nvr; i++) {
        const auto ref = vr[i];
        const auto idx = vrToStringIndices_.at(ref);
        stringBuffer_.emplace_back(strings_[idx].get());
        value[i] = stringBuffer_.back().c_str();
    }
}

void fmu_base::get_binary(const unsigned int vr[], size_t nvr, size_t valueSizes[], const uint8_t *values[]) {
    binaryBuffer_.clear();
    for (auto i = 0; i < nvr; i++) {
        const auto ref = vr[i];
        const auto idx = vrToBinaryIndices_.at(ref);
        const auto &data = binary_[idx].get();
        valueSizes[i] = data.size();
        binaryBuffer_.emplace_back(data.begin(), data.end());
        values[i] = binaryBuffer_.back().data();
    }
}

void fmu_base::set_integer(const unsigned int vr[], size_t nvr, const int value[]) {
    for (unsigned i = 0; i < nvr; i++) {
        const auto ref = vr[i];
        const auto idx = vrToIntegerIndices_.at(ref);
        integers_[idx].set(value[i]);
    }
}

void fmu_base::set_real(const unsigned int vr[], size_t nvr, const double value[]) {
    for (unsigned i = 0; i < nvr; i++) {
        const auto ref = vr[i];
        const auto idx = vrToRealIndices_.at(ref);
        reals_[idx].set(value[i]);
    }
}
void fmu_base::set_boolean(const unsigned int vr[], size_t nvr, const int value[]) {
    for (unsigned i = 0; i < nvr; i++) {
        const auto ref = vr[i];
        const auto idx = vrToBooleanIndices_.at(ref);
        booleans_[idx].set(static_cast<bool>(value[i]));
    }
}
void fmu_base::set_boolean(const unsigned int vr[], size_t nvr, const bool value[]) {
    for (unsigned i = 0; i < nvr; i++) {
        const auto ref = vr[i];
        const auto idx = vrToBooleanIndices_.at(ref);
        booleans_[idx].set(value[i]);
    }
}

void fmu_base::set_string(const unsigned int vr[], size_t nvr, const char *const value[]) {
    for (unsigned i = 0; i < nvr; i++) {
        const auto ref = vr[i];
        const auto idx = vrToStringIndices_.at(ref);
        strings_[idx].set(value[i]);
    }
}

void fmu_base::set_binary(const unsigned int vr[], size_t nvr, const size_t valueSizes[], const uint8_t *const value[]) {

    for (unsigned i = 0; i < nvr; i++) {
        const auto ref = vr[i];
        const auto idx = vrToBinaryIndices_.at(ref);
        const uint8_t *ptr = value[i];
        const size_t len = valueSizes[i];
        binary_[idx].set(std::vector(ptr, ptr + len));
    }
}


IntVariable &fmu_base::register_integer(const std::string &name, int *ptr, const std::function<void()> &onChange) {
    const auto vr = static_cast<unsigned int>(numVariables_++);
    auto &v = integers_.emplace_back(name, vr, numVariables_, ptr, onChange);
    vrToIntegerIndices_.emplace(v.value_reference(), integers_.size() - 1);
    return v;
}

IntVariable &fmu_base::register_integer(const std::string &name, const std::function<int()> &getter, const std::optional<std::function<void(int)>> &setter) {
    const auto vr = static_cast<unsigned int>(numVariables_++);
    auto &v = integers_.emplace_back(name, vr, numVariables_, getter, setter);
    vrToIntegerIndices_.emplace(v.value_reference(), integers_.size() - 1);
    return v;
}

RealVariable &fmu_base::register_real(const std::string &name, double *ptr, const std::function<void()> &onChange) {
    const auto vr = static_cast<unsigned int>(numVariables_++);
    auto &v = reals_.emplace_back(name, vr, numVariables_, ptr, onChange);
    vrToRealIndices_.emplace(v.value_reference(), reals_.size() - 1);
    return v;
}

RealVariable &fmu_base::register_real(const std::string &name, const std::function<double()> &getter, const std::optional<std::function<void(double)>> &setter) {
    const auto vr = static_cast<unsigned int>(numVariables_++);
    auto &v = reals_.emplace_back(name, vr, numVariables_, getter, setter);
    vrToRealIndices_.emplace(v.value_reference(), reals_.size() - 1);
    return v;
}

BoolVariable &fmu_base::register_boolean(const std::string &name, bool *ptr, const std::function<void()> &onChange) {
    const auto vr = static_cast<unsigned int>(numVariables_++);
    auto &v = booleans_.emplace_back(name, vr, numVariables_, ptr, onChange);
    vrToBooleanIndices_.emplace(v.value_reference(), booleans_.size() - 1);
    return v;
}

BoolVariable &fmu_base::register_boolean(const std::string &name, const std::function<bool()> &getter, const std::optional<std::function<void(bool)>> &setter) {
    const auto vr = static_cast<unsigned int>(numVariables_++);
    auto &v = booleans_.emplace_back(name, vr, numVariables_, getter, setter);
    vrToBooleanIndices_.emplace(v.value_reference(), booleans_.size() - 1);
    return v;
}

StringVariable &fmu_base::register_string(const std::string &name, std::string *ptr, const std::function<void()> &onChange) {
    const auto vr = static_cast<unsigned int>(numVariables_++);
    auto &v = strings_.emplace_back(name, vr, numVariables_, ptr, onChange);
    vrToStringIndices_.emplace(v.value_reference(), strings_.size() - 1);
    return v;
}

StringVariable &fmu_base::register_string(const std::string &name, const std::function<std::string()> &getter, const std::optional<std::function<void(std::string)>> &setter) {
    const auto vr = static_cast<unsigned int>(numVariables_++);
    auto &v = strings_.emplace_back(name, vr, numVariables_, getter, setter);
    vrToStringIndices_.emplace(v.value_reference(), strings_.size() - 1);
    return v;
}

BinaryVariable &fmu_base::register_binary(const std::string &name, BinaryType *ptr, const std::function<void()> &onChange) {
    const auto vr = static_cast<unsigned int>(numVariables_++);
    auto &v = binary_.emplace_back(name, vr, numVariables_, ptr, onChange);
    vrToBinaryIndices_.emplace(v.value_reference(), binary_.size() - 1);
    return v;
}

BinaryVariable &fmu_base::register_binary(const std::string &name, const std::function<BinaryType()> &getter, const std::optional<std::function<void(BinaryType)>> &setter) {
    const auto vr = static_cast<unsigned int>(numVariables_++);
    auto &v = binary_.emplace_back(name, vr, numVariables_, getter, setter);
    vrToBinaryIndices_.emplace(v.value_reference(), binary_.size() - 1);
    return v;
}

[[maybe_unused]] std::string fmu_base::guid() const {
    const model_info info = get_model_info();
    const std::vector content{
            info.author,
            info.version,
            info.description,
            info.modelName,
            model_identifier()};

    std::stringstream ss;
    for (const auto &str: content) {
        ss << str;
    }

    const auto vars = collect(integers_, reals_, booleans_, strings_);
    for (const auto &v: vars) {
        ss << v->name();
        ss << std::to_string(v->index());
        ss << std::to_string(v->value_reference());
        ss << to_string(v->causality());
        if (v->variability()) {
            ss << to_string(*v->variability());
        }
        if (v->initial()) {
            ss << to_string(*v->initial());
        }
    }

    return std::to_string(fnv1a(ss.str()));
}

void fmu_base::debugLog(const fmiStatus s, const std::string &message) const {
    if (data_.fmiLogger) {
        data_.fmiLogger->log(s, message);
    }
}

std::vector<unsigned> fmu_base::get_value_refs() const {
    std::vector<unsigned int> indices;
    const auto allVars = collect(integers_, reals_, booleans_, strings_);
    indices.reserve(allVars.size());
    for (const auto &v: allVars) {
        indices.emplace_back(v->value_reference());
    }

    return indices;
}


// An FMU state is the model's registered State plus the base clock. The
// clock has to travel with it: fmi2SetFMUstate rolls the co-simulation back
// to the time the state was taken, and the next fmi2DoStep arrives at that
// time, which step() checks against time_. Without the clock in the record a
// restored instance rejected its first step ("Current time does not match
// the internal time"), while modelDescription advertised
// canGetAndSetFMUstate. (Aetherion change to the vendored fmu4cpp; pre-DOI
// audit finding F05.) The serialized form is the clock's 8 bytes followed by
// the model's bytes.
namespace {
    struct fmu_state_record {
        double time;
        void *model;
    };
}

void *fmu_base::getFMUState() {
    if (!state_ops_ || !get_state_ptr_) throw fatal_error("getFMUState not implemented");
    const void *in_place = get_state_ptr_(this);
    return new fmu_state_record{time_, state_ops_->create_from_state(in_place)};
}

void fmu_base::setFmuState(void *state) {
    if (!state_ops_ || !get_state_ptr_) throw fatal_error("setFmuState not implemented");
    if (!state) throw fatal_error("setFmuState: null state");
    const auto *rec = static_cast<const fmu_state_record *>(state);
    void *dst = get_state_ptr_(this);
    state_ops_->assign_into_state(dst, rec->model);
    time_ = rec->time;
}

void fmu_base::freeFmuState(void **state) {
    if (!state_ops_ || !state || !*state) throw fatal_error("freeFmuState not implemented");
    auto *rec = static_cast<fmu_state_record *>(*state);
    state_ops_->destroy(rec->model);
    delete rec;
    *state = nullptr;
}

void fmu_base::serializedFMUStateSize(void *state, size_t &size) {
    if (!state_ops_) throw fatal_error("serializedFMUStateSize not implemented");
    (void) state;
    size = sizeof(double) + state_ops_->serialized_size();
}

void fmu_base::serializeFMUState(void *state, std::vector<uint8_t> &out) {
    if (!state_ops_) throw fatal_error("serializeFMUState not implemented");
    double time = time_;
    const void *model = get_state_ptr_(this);
    if (state) {
        const auto *rec = static_cast<const fmu_state_record *>(state);
        time = rec->time;
        model = rec->model;
    }
    std::vector<uint8_t> body;
    state_ops_->serialize(model, body);
    out.resize(sizeof(double) + body.size());
    std::memcpy(out.data(), &time, sizeof(double));
    std::memcpy(out.data() + sizeof(double), body.data(), body.size());
}

void fmu_base::deserializeFMUState(const std::vector<uint8_t> &in, void **out) {
    if (!state_ops_) throw fatal_error("deserializeFMUState not implemented");
    if (in.size() < sizeof(double) + state_ops_->serialized_size())
        throw fatal_error("deserializeFMUState: serialized state too short");
    double time = 0.0;
    std::memcpy(&time, in.data(), sizeof(double));
    const std::vector<uint8_t> body(in.begin() + sizeof(double), in.end());
    void *model = nullptr;
    state_ops_->deserialize(body, &model);
    *out = new fmu_state_record{time, model};
}

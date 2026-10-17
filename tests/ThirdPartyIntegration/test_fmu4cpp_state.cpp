// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// test_fmu4cpp_state.cpp
//
// Regression for the pre-DOI audit's finding F05, at the level of the
// vendored fmu4cpp base class, in process and without a co-simulation master.
//
// fmu_base::getFMUState copied only the model's registered state; the base
// clock stayed where the instance was. After fmi2SetFMUstate a master
// continues from the time the state was taken, and fmu_base::step() rejects
// a communication point that does not match its clock -- so the first step
// after a rollback threw "Current time does not match the internal time"
// on every FMU that advertised canGetAndSetFMUstate. The state record now
// carries the clock, in memory and in the serialized form.
// ------------------------------------------------------------------------------

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <fmu4cpp/fmu_base.hpp>

#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

    struct CounterState
    {
        int steps{ 0 };
        double integral{ 0.0 };
    };

    /// A model whose registered state is a step counter and the integral of 1
    /// over time; `clock` mirrors the base clock seen by do_step so a test can
    /// read it back through a registered output.
    class Counter : public fmu4cpp::fmu_base
    {
    public:
        FMU4CPP_CTOR(Counter)
        {
            register_integer("steps", &state_.steps).setCausality(fmu4cpp::causality_t::OUTPUT);
            register_real("integral", &state_.integral).setCausality(fmu4cpp::causality_t::OUTPUT);
            register_real("clock", &clock_).setCausality(fmu4cpp::causality_t::OUTPUT);
            register_state(&Counter::state_);
            Counter::reset();
        }

        bool do_step(double dt) override
        {
            ++state_.steps;
            state_.integral += dt;
            clock_ = currentTime() + dt;
            return true;
        }

        void reset() override
        {
            state_ = CounterState{};
            clock_ = 0.0;
        }

    private:
        CounterState state_{};
        double clock_{ 0.0 };
    };

} // namespace

fmu4cpp::model_info fmu4cpp::get_model_info() { return {}; }
std::string fmu4cpp::model_identifier() { return "counter_state_test"; }
FMU4CPP_INSTANTIATE(Counter);

namespace {
    double real(const fmu4cpp::fmu_base& m, const char* name)
    {
        const auto v = m.get_real_variable(name);
        if (!v.has_value()) throw std::runtime_error(std::string("no real variable ") + name);
        return v->get();
    }
    int integer(const fmu4cpp::fmu_base& m, const char* name)
    {
        const auto v = m.get_int_variable(name);
        if (!v.has_value()) throw std::runtime_error(std::string("no integer variable ") + name);
        return v->get();
    }
}

TEST_CASE("fmu4cpp: a restored FMU state restores the clock, so the next step at that time is accepted",
    "[fmu4cpp][state][regression]")
{
    auto m = fmu4cpp::createInstance({});
    m->enter_initialisation_mode(0.0, std::nullopt, std::nullopt);
    m->exit_initialisation_mode();

    REQUIRE(m->step(0.0, 0.1));
    REQUIRE(m->step(0.1, 0.1));
    void* checkpoint = m->getFMUState(); // t = 0.2, 2 steps

    REQUIRE(m->step(0.2, 0.1));
    REQUIRE(m->step(0.3, 0.1)); // t = 0.4, 4 steps
    CHECK(integer(*m, "steps") == 4);

    m->setFmuState(checkpoint);
    CHECK(integer(*m, "steps") == 2);
    CHECK(real(*m, "integral") == Catch::Approx(0.2));

    // The master continues from the checkpoint's time. This threw before.
    REQUIRE_NOTHROW(m->step(0.2, 0.1));
    CHECK(integer(*m, "steps") == 3);
    CHECK(real(*m, "clock") == Catch::Approx(0.3));

    // And a step at the pre-rollback time is now the wrong one.
    CHECK_THROWS_AS(m->step(0.4, 0.1), std::runtime_error);

    m->freeFmuState(&checkpoint);
    CHECK(checkpoint == nullptr);
}

TEST_CASE("fmu4cpp: a serialized FMU state carries the clock", "[fmu4cpp][state][regression]")
{
    auto m = fmu4cpp::createInstance({});
    m->enter_initialisation_mode(0.0, std::nullopt, std::nullopt);
    m->exit_initialisation_mode();
    for (int i = 0; i < 5; ++i) REQUIRE(m->step(0.1 * i, 0.1)); // t = 0.5

    void* s = m->getFMUState();
    size_t size = 0;
    m->serializedFMUStateSize(s, size);
    CHECK(size == sizeof(double) + sizeof(CounterState));

    std::vector<std::uint8_t> bytes;
    m->serializeFMUState(s, bytes);
    REQUIRE(bytes.size() == size);
    m->freeFmuState(&s);

    // Move on, then restore from the bytes into a fresh instance.
    REQUIRE(m->step(0.5, 0.1));
    auto n = fmu4cpp::createInstance({});
    n->enter_initialisation_mode(0.0, std::nullopt, std::nullopt);
    n->exit_initialisation_mode();
    void* r = nullptr;
    n->deserializeFMUState(bytes, &r);
    n->setFmuState(r);
    n->freeFmuState(&r);

    CHECK(integer(*n, "steps") == 5);
    CHECK(real(*n, "integral") == Catch::Approx(0.5));
    REQUIRE_NOTHROW(n->step(0.5, 0.1)); // the clock came with the bytes
    CHECK(integer(*n, "steps") == 6);

    // The current state, serialized without a handle, carries the clock too.
    std::vector<std::uint8_t> now;
    m->serializeFMUState(nullptr, now);
    double t = 0.0;
    std::memcpy(&t, now.data(), sizeof(double));
    CHECK(t == Catch::Approx(0.6));
}

// Cap'n Proto free shim of c8/io/capnp-messages.h (US-018).
//
// faces/face-tracker.cc:243-246 builds a default message only to pass its reader on:
//   ConstRootMessage<DebugOptions> opts; return track(..., opts.reader());
// This shim keeps that call surface: a default ConstRootMessage<T> yields a default T::Reader
// (schema defaults, as a default capnp root message does). Nothing else of the original header is
// used by the vendored closure.
#pragma once

namespace c8 {

template <typename T>
class ConstRootMessage {
public:
  ConstRootMessage() = default;
  typename T::Reader reader() const { return typename T::Reader(); }
};

}  // namespace c8

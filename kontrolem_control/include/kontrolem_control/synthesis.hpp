// Kontrol'Em v2 — the artifact produced by the offline synthesize() step.
//
// Synthesis is the serializable product of synthesize(), consumed by
// configure(). It is polymorphic so each paradigm stores exactly what it needs:
//   LQR : a real artifact (gain K + operating point) -> derived LqrSynthesis.
//   QP  : nothing to precompute (online-solved)      -> the empty base itself.
// This is where the offline/online seam lives: heavy design math produces a
// Synthesis once; the runtime only ever loads one and calls compute().
#ifndef KONTROLEM_CONTROL__SYNTHESIS_HPP_
#define KONTROLEM_CONTROL__SYNTHESIS_HPP_

namespace kontrolem_control
{

/// Base of the synthesized artifact. Concrete controllers derive their own
/// (and downcast in configure()); a controller with nothing to precompute
/// returns an instance of this base.
struct Synthesis
{
  virtual ~Synthesis() = default;
};

}  // namespace kontrolem_control

#endif  // KONTROLEM_CONTROL__SYNTHESIS_HPP_

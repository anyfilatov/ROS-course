
template <typename F>
class Cleanup final
{
public:
  Cleanup(F f) noexcept : f_(f) {}
  ~Cleanup() noexcept { f_(); }
private:
  F f_;
};

template <typename F>
Cleanup<F> cleanup(F f) noexcept
{
  return Cleanup<F>(f);
}

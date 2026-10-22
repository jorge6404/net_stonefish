#include "damage.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <random>
#include <set>
#include <utility>
#include <vector>

#include "graph.hpp"

namespace stonefish_net::core::topology
{
namespace
{
using Cell = std::pair<int, int>;  // rombo centrado en (ci, cj), ci + cj impar

class Damager
{
public:
  Damager(const NetConfig& c, const Lattice& lat)
  : c_(c), lat_(lat), rng_(static_cast<std::mt19937_64::result_type>(c.damage.seed))
  {
  }

  std::set<Edge> run(const std::vector<Link>& links, DamageReport& report)
  {
    std::vector<Cell> candidates;
    for (int cj = 2; cj <= lat_.nJ - 2; ++cj) {
      for (int ci = 2; ci <= lat_.nI - 2; ++ci) {
        if (eligible({ci, cj})) candidates.push_back({ci, cj});
      }
    }
    for (int k = 0; k < c_.damage.tears && !candidates.empty(); ++k) {
      tear(candidates, report);
    }
    cut_loose(links, report);
    return std::move(removed_);
  }

private:
  int uniform(int lo, int hi) { return std::uniform_int_distribution<int>(lo, hi)(rng_); }
  double unit() { return std::uniform_real_distribution<double>(0.0, 1.0)(rng_); }

  // Rombos que no tocan el marco (así el borde queda siempre intacto)
  bool eligible(const Cell& cell) const
  {
    const auto [ci, cj] = cell;
    return (ci + cj) % 2 == 1 && ci >= 2 && ci <= lat_.nI - 2 && cj >= 2 && cj <= lat_.nJ - 2;
  }

  // Los 4 hilos del rombo unen los nudos vecinos en cruz
  std::vector<Edge> cell_edges(const Cell& cell) const
  {
    const auto [ci, cj] = cell;
    const int l = lat_.at(ci - 1, cj), u = lat_.at(ci, cj - 1), r = lat_.at(ci + 1, cj), d = lat_.at(ci, cj + 1);
    if (l < 0 || u < 0 || r < 0 || d < 0) {
      return {};
    }
    return {edge(l, u), edge(u, r), edge(r, d), edge(d, l)};
  }

  void tear(const std::vector<Cell>& candidates, DamageReport& report)
  {
    Cell start;
    int tries = 0;
    do {
      start = candidates[uniform(0, static_cast<int>(candidates.size()) - 1)];
    } while (torn_.count(start) && ++tries < 100);
    if (torn_.count(start)) {
      return;
    }
    const int size = uniform(c_.damage.tear_min_meshes, c_.damage.tear_max_meshes);
    std::vector<Cell> region{start};
    std::set<Cell> in{start};
    while (static_cast<int>(region.size()) < size) {
      std::vector<Cell> frontier;
      for (const auto& [ci, cj] : region) {
        for (auto [di, dj] : {std::pair{1, 1}, std::pair{1, -1}, std::pair{-1, 1}, std::pair{-1, -1}}) {
          const Cell n{ci + di, cj + dj};
          if (eligible(n) && !in.count(n) && !torn_.count(n)) frontier.push_back(n);
        }
      }
      std::sort(frontier.begin(), frontier.end());
      frontier.erase(std::unique(frontier.begin(), frontier.end()), frontier.end());
      if (frontier.empty()) {
        break;
      }
      const Cell next = frontier[uniform(0, static_cast<int>(frontier.size()) - 1)];
      region.push_back(next);
      in.insert(next);
    }
    // Hilos interiores (compartidos por dos rombos del roto) fuera; los del anillo exterior se quedan.
    std::map<Edge, int> uses;
    for (const Cell& cell : region) {
      for (const Edge& e : cell_edges(cell)) ++uses[e];
    }
    std::set<int> ring;  // nudos que conservan algún hilo del borde del roto
    for (const auto& [e, n] : uses) {
      if (n == 1) {
        ring.insert(e.first);
        ring.insert(e.second);
      }
    }
    for (const auto& [e, n] : uses) {
      if (n < 2) {
        continue;
      }
      // cabo suelto: hilo interior con un extremo en el anillo, colgando de él
      const bool stub = (ring.count(e.first) != ring.count(e.second)) && unit() < c_.damage.stub_fraction;
      if (!stub) removed_.insert(e);
    }
    torn_.insert(region.begin(), region.end());
    report.tear_sizes.push_back(static_cast<int>(region.size()));
  }

  // Fracción de los hilos de la red intacta, elegidos entre los que siguen en pie (Fisher–Yates parcial)
  void cut_loose(const std::vector<Link>& links, DamageReport& report)
  {
    std::vector<Edge> standing;
    int intact = 0;
    for (const auto& l : links) {
      if (l.kind == LinkKind::Twine) {
        ++intact;
        if (!removed_.count(edge(l.a, l.b))) standing.push_back(edge(l.a, l.b));
      }
    }
    const int want = std::min(static_cast<int>(std::lround(c_.damage.loose_fraction * intact)),
                              static_cast<int>(standing.size()));
    for (int k = 0; k < want; ++k) {
      std::swap(standing[k], standing[uniform(k, static_cast<int>(standing.size()) - 1)]);
      removed_.insert(standing[k]);
    }
    report.loose_cut = want;
  }

  const NetConfig& c_;
  const Lattice& lat_;
  std::mt19937_64 rng_;
  std::set<Cell> torn_;
  std::set<Edge> removed_;
};
}  // namespace

void add_damage(const NetConfig& c, const Lattice& lat, NetTopology& t)
{
  const std::set<Edge> removed = Damager(c, lat).run(t.links, t.damage);
  t.links.erase(std::remove_if(t.links.begin(), t.links.end(),
                               [&](const Link& l) { return l.kind == LinkKind::Twine && removed.count(edge(l.a, l.b)); }),
                t.links.end());
}

}  // namespace stonefish_net::core::topology

#include "graph.hpp"

#include <algorithm>
#include <cstdint>
#include <unordered_set>
#include <vector>

namespace stonefish_net::core::topology
{

namespace
{

// Hash simétrico en O(1) para aristas no dirigidas.
// Si Edge es std::pair<int, int>, usa .first y .second; si es un struct propio, usa .a y .b.
struct EdgeHash
{
  std::size_t operator()(const Edge& e) const noexcept
  {
    // Asumiendo que la función auxiliar edge(a, b) ya ordena los índices (min, max).
    // Si Edge usa .first/.second, cámbialo aquí según tu definición en graph.hpp:
    const auto u = static_cast<std::uint64_t>(e.first);
    const auto v = static_cast<std::uint64_t>(e.second);
    return static_cast<std::size_t>((u << 32) ^ v);
  }
};

}  // namespace

int add_node(NetTopology& t, const Eigen::Vector3d& position, NodeKind kind)
{
  t.nodes.emplace_back(Node{position, kind});
  return static_cast<int>(t.nodes.size()) - 1;
}

void add_link(NetTopology& t, int a, int b, double diameter, LinkKind kind)
{
  const int n = static_cast<int>(t.nodes.size());

  // Validaciones de límites, auto-bucles y parámetros físicos
  if (a < 0 || a >= n || b < 0 || b >= n || a == b || diameter <= 0.0) {
    return;
  }

  const double length = (t.nodes[a].position - t.nodes[b].position).norm();
  if (length <= 1e-9) {
    return;
  }

  t.links.emplace_back(Link{a, b, length, diameter, kind});
}

void prune_to_component(NetTopology& t, int root)
{
  const int n = static_cast<int>(t.nodes.size());

  // 1. Validación de límites para root
  if (root < 0 || root >= n) {
    t.nodes.clear();
    t.links.clear();
    t.strands.clear();
    std::vector<int> empty_map(n, -1);
    t.marks.remap(empty_map);
    return;
  }

  // 2. Construcción de lista de adyacencia segura
  std::vector<std::vector<int>> adj(n);
  for (const auto& l : t.links) {
    if (l.a >= 0 && l.a < n && l.b >= 0 && l.b < n) {
      adj[l.a].push_back(l.b);
      adj[l.b].push_back(l.a);
    }
  }

  // 3. Recorrido DFS iterativo
  std::vector<bool> keep(n, false);
  std::vector<int> stack;
  stack.reserve(n);
  stack.push_back(root);
  keep[root] = true;

  int kept_count = 0;
  while (!stack.empty()) {
    const int k = stack.back();
    stack.pop_back();
    ++kept_count;

    for (int v : adj[k]) {
      if (!keep[v]) {
        keep[v] = true;
        stack.push_back(v);
      }
    }
  }

  // 4. Reindexado y compactación de nodos
  std::vector<int> map(n, -1);
  std::vector<Node> nodes;
  nodes.reserve(kept_count);

  for (int k = 0; k < n; ++k) {
    if (keep[k]) {
      map[k] = static_cast<int>(nodes.size());
      nodes.push_back(t.nodes[k]);
    }
  }

  // 5. Filtrado de enlaces y registro en tabla hash O(1)
  std::unordered_set<Edge, EdgeHash> present;
  present.reserve(t.links.size());

  std::vector<Link> links;
  links.reserve(t.links.size());

  for (auto l : t.links) {
    if (l.a >= 0 && l.a < n && l.b >= 0 && l.b < n && keep[l.a] && keep[l.b]) {
      l.a = map[l.a];
      l.b = map[l.b];
      present.insert(edge(l.a, l.b));
      links.push_back(l);
    }
  }

  // 6. Hilos dibujados: se parten donde falta un tramo
  std::vector<Strand> strands;
  strands.reserve(t.strands.size());

  for (const auto& s : t.strands) {
    Strand run{{}, s.kind};
    auto flush = [&]() {
      if (run.nodes.size() >= 2) {
        strands.push_back(std::move(run));
      }
      run.nodes.clear();
    };

    for (int old : s.nodes) {
      if (old < 0 || old >= n) {
        flush();
        continue;
      }

      const int v = map[old];
      if (v < 0) {
        flush();
        continue;
      }

      if (!run.nodes.empty() && present.find(edge(run.nodes.back(), v)) == present.end()) {
        flush();
      }
      run.nodes.push_back(v);
    }
    flush();
  }

  // 7. Reemplazo mediante semántica de movimiento
  t.nodes = std::move(nodes);
  t.links = std::move(links);
  t.strands = std::move(strands);
  t.marks.remap(map);
}

}  // namespace stonefish_net::core::topology
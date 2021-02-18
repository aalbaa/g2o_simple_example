// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "edge_R2.h"

using namespace Eigen;

namespace g2o {
  namespace tutorial {

    EdgeR2::EdgeR2() :
      BaseBinaryEdge<2, Vector2, VertexR2, VertexR2>()
    {
        // Set up the system matrices
        A(0, 0) = 0;
        A(0, 1) = 1;
        A(1, 0) = -1;
        A(1, 1) = -1;

        B(0, 0) = 0;
        B(0, 1) = 0;
        B(1, 0) = 1;
        B(1, 1) = 0;
    }

    bool EdgeR2::read(std::istream& is)
    {
      Vector2d p;
      is >> p[0] >> p[1];
      _measurement = p;
      _inverseMeasurement = measurement().inverse();
      for (int i = 0; i < 2; ++i)
        for (int j = i; j < 2; ++j) {
          is >> information()(i, j);
          if (i != j)
            information()(j, i) = information()(i, j);
        }
      return true;
    }

    bool EdgeR2::write(std::ostream& os) const
    {
      Vector2d p = measurement();
      os << p[0] << " " << p[1];
    //   os << p.x() << " " << p.y() << " " << p.z();
      for (int i = 0; i < 2; ++i)
        for (int j = i; j < 2; ++j)
          os << " " << information()(i, j);
      return os.good();
    }
  } // end namespace
} // end namespace

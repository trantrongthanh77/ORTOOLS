
%module constraint_solver
%{
#include "constraint_solver.h"
%}
%include "std_string.i"
%include "std_vector.i"
%include "constraint_solver.h"

namespace std {
    %template(DoubleVector) vector<double>;
}

namespace std {
    %template(DistanceMatrix) vector<vector<double>>;
}

namespace std {
    %template(IntVector) vector<int>;
}

namespace std {
    %template(PairIntVector) vector<vector<int>>;
}

%insert(cgo_comment_typedefs) %{
#cgo LDFLAGS: -L../lib -lortools
#cgo CPPFLAGS: -I../include
%}

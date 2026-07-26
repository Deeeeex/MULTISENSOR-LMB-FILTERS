function adjacency = ...
    convertDiagnosticEdgeMaskToPolicyAdjacency(diagnosticEdgeMask)
% CONVERTDIAGNOSTICEDGEMASKTOPOLICYADJACENCY Align directed conventions.
%
% Filter diagnostics store a directed message as
%   diagnosticEdgeMask(sender, receiver) = true.
% Topology policies consume
%   adjacency(receiver, sender) = true.

if ndims(diagnosticEdgeMask) ~= 2 || ...
        size(diagnosticEdgeMask, 1) ~= size(diagnosticEdgeMask, 2)
    error('The diagnostic directed-edge mask must be square.');
end
adjacency = logical(diagnosticEdgeMask');
nodeCount = size(adjacency, 1);
adjacency(1:nodeCount+1:end) = false;
end

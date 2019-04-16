function [fusedArtifacts] = fuseArtifacts(artifacts, fusedArtifacts)
for artifact = artifacts
    addArtifact = true;
    for fartifact = fusedArtifacts
        if artifact.pos == fartifact.pos
            addArtifact = false;
            break;
        end
    end
    if addArtifact
        fusedArtifacts(end+1) = artifact;
    end
end
end


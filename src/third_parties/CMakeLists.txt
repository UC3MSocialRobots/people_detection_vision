### libdai
SET(DAI "libdai/src")
ADD_LIBRARY(libdai
    ${DAI}/alldai.cpp
    ${DAI}/bbp.cpp
    ${DAI}/bipgraph.cpp
    ${DAI}/bp.cpp
    ${DAI}/bp_dual.cpp
    ${DAI}/cbp.cpp
    ${DAI}/clustergraph.cpp
    ${DAI}/cobwebgraph.cpp
    ${DAI}/dag.cpp
    ${DAI}/daialg.cpp
    ${DAI}/decmap.cpp
    ${DAI}/emalg.cpp
    ${DAI}/evidence.cpp
    ${DAI}/exactinf.cpp
    ${DAI}/exceptions.cpp
    ${DAI}/factor.cpp
    ${DAI}/factorgraph.cpp
    ${DAI}/fbp.cpp
    ${DAI}/gibbs.cpp
    ${DAI}/glc.cpp
    ${DAI}/graph.cpp
    ${DAI}/hak.cpp
    ${DAI}/io.cpp
    ${DAI}/jtree.cpp
    ${DAI}/lc.cpp
    ${DAI}/mf.cpp
    ${DAI}/mr.cpp
    ${DAI}/properties.cpp
    ${DAI}/regiongraph.cpp
    ${DAI}/treeep.cpp
    ${DAI}/trwbp.cpp
    ${DAI}/util.cpp
    ${DAI}/varset.cpp
    ${DAI}/weightedgraph.cpp
)
SET_TARGET_PROPERTIES(libdai PROPERTIES
  COMPILE_FLAGS "-DDAI_DEBUG  -DDAI_WITH_BP -DDAI_WITH_FBP -DDAI_WITH_TRWBP -DDAI_WITH_MF -DDAI_WITH_HAK -DDAI_WITH_LC -DDAI_WITH_TREEEP -DDAI_WITH_JTREE -DDAI_WITH_MR -DDAI_WITH_GIBBS -DDAI_WITH_CBP -DDAI_WITH_DECMAP -DDAI_WITH_GLC"
)
TARGET_LINK_LIBRARIES(libdai gmpxx gmp)

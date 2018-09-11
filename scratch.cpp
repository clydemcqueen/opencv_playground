// diag is 1
// everything else is 0
// then these:

transferFunction_(StateMemberX, StateMemberVx) = delta;
transferFunction_(StateMemberY, StateMemberVy) = delta;
transferFunction_(StateMemberZ, StateMemberVz) = delta;

transferFunction_(StateMemberVx, StateMemberAx) = delta;
transferFunction_(StateMemberVy, StateMemberAy) = delta;
transferFunction_(StateMemberVz, StateMemberAz) = delta;

transferFunction_(StateMemberX, StateMemberAx) = 0.5 * delta * delta;
transferFunction_(StateMemberY, StateMemberAy) = 0.5 * delta * delta;
transferFunction_(StateMemberZ, StateMemberAz) = 0.5 * delta * delta;
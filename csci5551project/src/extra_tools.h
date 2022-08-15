#ifndef MY_GRAVEYARD // include guard
#define MY_GRAVEYARD

class quadNode {
        public:
                quadNode(float, float);
                quadNode * nw;
                quadNode * sw;
                quadNode * ne;
                quadNode * se;
                float myX;
                float myY;
                bool addNode(float, float);
};

quadNode::quadNode (float thisX, float thisY) {
        myX = thisX;
        myY = thisY;
        nw = sw = ne = se = NULL;
}

bool quadNode::addNode(float theirX, float theirY) {
        if ((theirX < myX) && (theirY > myY)) {
                if (nw == NULL) {
                        quadNode * theirNode = new quadNode(theirX, theirY);
                        nw = theirNode;
                        return true;
                }
                else {
                        return nw->addNode(theirX, theirY);
                }
        }
        else if ((theirX < myX) && (theirY < myY)) {
                if (sw == NULL) {
                        quadNode * theirNode = new quadNode(theirX, theirY);
                        sw = theirNode;
                        return true;
                }
                else {
                        return sw->addNode(theirX, theirY);
                }
        }
        else if ((theirX > myX) && (theirY < myY)) {
                if (se == NULL) {
                        quadNode * theirNode = new quadNode(theirX, theirY);
                        se = theirNode;
                        return true;
                }
                else {
                        return se->addNode(theirX, theirY);
                }
        }
        else if ((theirX > myX) && (theirY > myY)) {
                if (ne == NULL) {
                        quadNode * theirNode = new quadNode(theirX, theirY);
                        ne = theirNode;
                        return true;
                }
                else {
                        return ne->addNode(theirX, theirY);
                }
        }
        else {
                return false;
        }
}

void printNode(quadNode curNode) {
        if(curNode.nw != NULL) {
                printNode(*curNode.nw);
        }
        if(curNode.sw != NULL) {
                printNode(*curNode.sw);
        }
        if(curNode.se != NULL) {
                printNode(*curNode.se);
        }
        if(curNode.ne != NULL) {
                printNode(*curNode.ne);
        }
        std::cout << "(" << curNode.myX << ", " << curNode.myY << ")\n";
}

//Make sure that x, y, and yaw are from same time
class posTriple {
        public:
                float curX;
                float curY;
                float curYaw;

                posTriple();
                void addX(float);
                void addY(float);
                void addYaw(float);
                void reset();
                bool isFull();
};

posTriple::posTriple() {
        float myNanf = nanf("");
        curX = myNanf;
        curY = myNanf;
        curYaw = myNanf;
}

void posTriple::addX(float thisX) {
        if(isnan(this->curX)) {
                curX = thisX;
        }
}

void posTriple::addY(float thisY) {
        if(isnan(this->curY)) {
                curY = thisY;
        }
}

void posTriple::addYaw(float thisYaw) {
        if(isnan(this->curYaw)) {
                curYaw = thisYaw;
        }
}

void posTriple::reset() {
        float myNanf = nanf("");
        this->curX = myNanf;
        this->curY = myNanf;
        this->curYaw = myNanf;
}

bool posTriple::isFull() {
        if(isnan(this->curX) || isnan(this->curY) || isnan(this->curYaw)) {
                return false;
        }
        else {
                return true;
        }
}

#endif

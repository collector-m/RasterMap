#ifndef POS_H_INCLUDED
#define POS_H_INCLUDED

typedef struct cluster_pos
{
    unsigned int id;
    int row;
    int col;
} cluster_pos_t;

class pos
{
public:
    int row, col;

    bool operator < (const pos& rpos) const
    {
        return (this->row < rpos.row) || ( (this->row == rpos.row) && (this->col < rpos.col) );
//        if(this->row < rpos.row) {
//            return true;
//        } else if(this->row == rpos.row) {
//            if(this->col < rpos.col) {
//                return true;
//            } else {
//                return false;
//            }
//        } else {
//            return false;
//        }

    }
};

inline bool operator == (const pos& lpos, const pos& rpos)
{
    return (lpos.row == rpos.row) && (lpos.col == rpos.col);
}

inline bool operator != (const pos& lpos, const pos& rpos)
{
    return !(lpos == rpos);
}

//inline bool operator < (const pos& lpos, const pos& rpos)
//{
//    if(lpos.row < rpos.row) {
//        return true;
//    } else if(lpos.row == rpos.row) {
//        if(lpos.col < rpos.col) {
//            return true;
//        } else {
//            return false;
//        }
//    } else {
//        return false;
//    }
//}

#endif // POS_H_INCLUDED

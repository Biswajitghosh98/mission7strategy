#ifndef STRATEGY_H
#define STRATEGY_H

typedef struct {
	int x;
	int y;
}point;

class strategy{

public:
	float angle(float ang);
	float distwhitel(point bot,float orinet);
	void action(point t_bot,point bot,float orient);
	void plan(int no,vector<int> a);
};

#endif
void SetYear(sm)
register struct of_shared_mem *sm;
{
	static short int days[12] = {0, 31, 59, 90, 120, 151,
			181, 212, 243, 273, 304, 334};
	double dFractYear;	/* Difference in clocks in fract of a year */
	struct timeval tv;
	struct timezone tz;

	/* read lynx system clock (tv_sec contains seconds since 1/1/70) */
	gettimeofday(&tv, &tz);

	/* get the closest year from the lynx clock */
	sm->cl.year = tv.tv_sec / (SEC_PER_DAY * 365.25);
	dFractYear = tv.tv_sec / (SEC_PER_DAY * 365.25) - sm->cl.year -
		(sm->cl.day + days[sm->cl.month - 1]) / 365.25;
	if(dFractYear > 0.5)
		sm->cl.year += 1971;
	else if(dFractYear < -0.5)
		sm->cl.year += 1969;
	else
		sm->cl.year += 1970;
}

/*
 * Calculate the Julian day for noon today.
 * Taken from Hughes, Yallop, and Hohenkerk MNRAS 238,1529 (1989) 
 */
int JulDay(y, m, d)
int y, m, d;
{
	if(m > 2) {
		m -= 3;
	} else {
		y -= 1;
		m += 9;
	}
	return(365 * y + (y >> 2) + (int)(30.6 * m + 0.5) + d + 1721155
		- (3 * (49 + y / 100)) / 4);
}

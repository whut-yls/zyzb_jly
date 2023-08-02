
#include "StringTransform.h"


/*short ing average */
uint16_t Gets_Average(uint16_t *pt,int l)
{
	int i,min,max,sum;
	min = pt[0];
	max = pt[0];
	sum = pt[0];
	for(i=1;i<l;i++)
	{
		
		if(pt[i]>max) max = pt[i];
		if(pt[i]<min) min = pt[i];
		sum += pt[i];
	}
	return (sum-min-max)/(l-2);
}

/*float average */
float Getf_Average(float *pt,int l)
{
	int i	;
	float min,max,sum;
	min = pt[0];
	max = pt[0];
	sum = pt[0];
	for(i=1;i<l;i++)
	{
		
		if(pt[i]>max) max = pt[i];
		if(pt[i]<min) min = pt[i];
		sum += pt[i];
	}
	return (sum-min-max)/(l-2);
}


int IsString(char src)
{
	if((src<'A') || (src>'Z' && src < 'a') || (src>'z'))
	{
		return 0;
	}
	else
	{
		return 1;
	}
}
void strupper(char* src,int l)
{
	int i;

	for (i = 0; i < l; ++i)
		{
			if(src[i]>='a' && src[i]<='z')
				src[i]-=32;
		}
}

void strlower(char* src,int l)
{
	int i;

	for (i = 0; i < l; ++i)
		{
			if(src[i]>='A' && src[i]<='Z')
				src[i]+=32;
		}
}

/* 
	transform hex string to byte binary

	return transfrom bytes

*/
int  AscToHex(char *a, unsigned char *t, int l)
{
	int i,te; 
	char temp;
	unsigned char	th,tl;
	
	for(i=0,te=0; i<l; )
	{
		th=0;
		tl=0;
		temp = *(a+i);
		if(temp >= '0' && temp <= '9')
			th = temp-'0';
		else if((temp|0x20) >= 'a' && (temp|0x20) <= 'f')
		{
			th = temp-'a'+0x0A;
		}
		else
			return te;
		
		++i;
		temp = *(a+i);
		if(temp >= '0' && temp <= '9')
			tl = temp-'0';
		else if((temp|0x20) >= 'a' && (temp|0x20) <= 'f')
		{
			tl = temp-'a'+0X0A;
		}
		else 
			return te;
		++i;
		*(t+te) = (th<<4)|tl;
		te++;
	}

	return te;
}

/* pasere time format +CTZE: “+32”,0,“2013/08/23,06:51:13”
*/
int CTZEToTime(char *r,int l, 
										unsigned char *year,
										unsigned char *month,
										unsigned char *date,
										unsigned char *hour,
										unsigned char *minute,
										unsigned char *Second)
{
	int i;
	char *p;
	*(r+l-1) = '\0';
	if((p=memchr(r,'\"',l))!=NULL)
	{
		p +=1;
		i = atoi(p)/4;

	}
	else
	{
		return 0;
	}

	if((p=strstr(p,",\""))!=NULL)
	{
		p +=2;
		*year = atoi(p)-2000;
		if(*year >99)
		{
			return 0;
		}
		
	}
	else
	{
		return 0;
	}
	if((p=strstr(p,"/"))!=NULL)
	{
		p +=1;
		*month = atoi(p);
		if(*month >12 || *month==0)
		{
			return 0;
		}
		
	}
	else
	{
		return 0;
	}

	if((p=strstr(p,"/"))!=NULL)
	{
		p +=1;
		*date = atoi(p);
		if(*date >31 || *date==0)
		{
			return 0;
		}
		
	}
	else
	{
		return 0;
	}

	if((p=strstr(p,","))!=NULL)
	{
		p +=1;
		*hour = atoi(p)+i;
		if(*hour >23)
		{
			return 0;
		}
		
	}
	else
	{
		return 0;
	}
	if((p=strstr(p,":"))!=NULL)
	{
		p +=1;
		*minute = atoi(p);
		if(*minute >59 )
		{
			return 0;
		}
		
	}
	else
	{
		return 0;
	}
	if((p=strstr(p,":"))!=NULL)
	{
		p +=1;
		*Second = atoi(p);
		if(*Second >59)
		{
			return 0;
		}
		
	}
	else
	{
		return 0;
	}

	return 1;
	
}

/* pasere time format +QLTS: "13/08/23,06:51:13+32,0"
*/
int QLTSToTime(char *r,int l, 
										unsigned char *year,
										unsigned char *month,
										unsigned char *date,
										unsigned char *hour,
										unsigned char *minute,
										unsigned char *Second)
{
	int i;
	char *p;
	*(r+l-1) = '\0';
	if((p=memchr(r,'\"',l))!=NULL)
	{
		p +=1;
		i	 = atoi(p);
		if(i>2000 && i<2099)
		{
			*year = i-2000;
		}
		else if(*year <99 && *year > 0)
		{
			*year = i;
		}
		else
		{
				return 0;
		}

	}
	else
	{
		return 0;
	}


	if((p=strstr(p,"/"))!=NULL)
	{
		p +=1;
		*month = atoi(p);
		if(*month >12 || *month==0)
		{
			return 0;
		}
		
	}
	else
	{
		return 0;
	}

	if((p=strstr(p,"/"))!=NULL)
	{
		p +=1;
		*date = atoi(p);
		if(*date >31 || *date==0)
		{
			return 0;
		}
		
	}
	else
	{
		return 0;
	}

	if((p=strstr(p,","))!=NULL)
	{
		p +=1;
		*hour = atoi(p);
	}
	else
	{
		return 0;
	}
	if((p=strstr(p,":"))!=NULL)
	{
		p +=1;
		*minute = atoi(p);
		if(*minute >59 )
		{
			return 0;
		}
		
	}
	else
	{
		return 0;
	}
	if((p=strstr(p,":"))!=NULL)
	{
		p +=1;
		*Second = atoi(p);
		if(*Second >59)
		{
			return 0;
		}
		
	}
	else
	{
		return 0;
	}

	p +=2;
	i = atoi(p)/4;
	*hour += i;
	if(*hour >= 24)
	{
		return 0;
	}
	
	return 1;
	
}


#include <nlist.h>
#include <sensor_data.h>
#include <stdio.h>
#include <stdlib.h>

static char *map_path;

void map_init(char *m_path)
{
map_path=m_path;
}

void map_render(beacon_t b[], int b_cnt, beacon_t m[], int m_cnt )
{
int i;
FILE *fp;
char description[1024];

fp=fopen(map_path,"w" );
if(fp==NULL ) 
	{
	printf( "Can't open %s for writing\n",map_path );
	return;
	}
fprintf(fp,"<?xml version=\"1.0\" encoding=\"utf-8\"?>" );
fprintf(fp,"<feed version=\"2.0\" xmlns:media=\"http://search.yahoo.com/mrss/\" xmlns:dc=\"http://purl.org/dc/elements/1.1/\" xmlns:geo=\"http://www.w3.org/2003/01/geo/wgs84_pos#\" xmlns:georss=\"http://www.georss.org/georss\">\n" );


for(i=0; i< b_cnt; i++ )
{
fprintf( fp,"<entry>\n<title>Node Id: 0x%X</title>",b[i].mac );
fprintf( fp,"<geo:long>%d</geo:long>",b[i].x );
fprintf( fp,"<geo:lat>%d</geo:lat>",b[i].y );
// agr XXX removed sensor data printing to map...
//if( sensor_data_get(b[i].mac, description) == 1 )
//	fprintf(fp,"<description> %s </description>", description );
fprintf( fp,"<media:thumbnail url=\"./icons/green-marker.gif\" height=\"100\" width=\"100\" /> </entry> " );
}

for(i=0; i< m_cnt; i++ )
{
if(m[i].desc[0]!='X' )
	{
	fprintf( fp,"<entry>\n<title>Mobile Node: 0x%X</title>",m[i].mac );

// agr XXX removed sensor data printing to map...
	//	if( sensor_data_get(m[i].mac, description) == 1 ) fprintf(fp,"<description>%s</description>", description );
	fprintf( fp,"<geo:long>%d</geo:long>",m[i].x );
	fprintf( fp,"<geo:lat>%d</geo:lat>",m[i].y );
	if(m[i].desc[0]=='S')
	fprintf( fp,"<media:thumbnail url=\"./icons/red-flag.gif\" height=\"100\" width=\"100\" /> </entry> " );
	else fprintf( fp,"<media:thumbnail url=\"./icons/red-tack.gif\" height=\"100\" width=\"100\" /> </entry> " );
	}
}


fprintf( fp,"</feed>" );
fclose(fp);
}

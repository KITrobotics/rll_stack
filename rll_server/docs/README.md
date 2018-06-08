 
### RLL API Description

API description done with Swagger aka OpenAPI.

For more information about OpenAPI, see: https://swagger.io/docs/specification/about/


#### Instructions to view with Swagger-ui
In order to get the API description for the RLL, by now its the easiest way to tun an a docker container.

Just run

	docker run -p 9090:8080 -e SWAGGER_JSON=/tmp/openapi.yaml -v /home/my_user/path/to/rll_server/docs:/tmp swaggerapi/swagger-ui
	
**Important: Make sure you set the right path, if not you will see the pet shop example of swagger**

Then navigate to localhost:9090 with your browser of choice and you should see the swagger-ui with the RLL description.

#### Development

In order to improve the API Description you can use the online swagger editor, copy and paste your openapi.yaml.

Its also possible to run your own editor in docker: 

	docker run -d -p 80:8080 swaggerapi/swagger-editor
	
See:
	https://github.com/swagger-api/swagger-editor

Also a nice tool for expanding the api-description:
	https://openapi-map.apihandyman.io/

#### Improvements with Dapperdox
In order to allow fusion of the api description from swagger and markdown, we can use a project called dapperdox: http://dapperdox.io/docs/overview

In order to run it, download the latest version from the project page. You find the binary in the folder that you can execute with the proper parameters:

	./dapperdox \
  	 -spec-dir=/home/my_user/rll_server/docs/dapperdox/specification \
    -assets-dir=/home/my_user/rll_server/docs/dapperdox/assets \
    -bind-addr=0.0.0.0:3123 \
    -site-url=http://localhost:3123 \
    -log-level=trace \
    -force-specification-list=false \
    -theme=default  \
    
   Then you should be able to see dapperbox on http://localhost:3123.
   
##### Edit dapperbox sites
In order to edit the dapperbox sites we can use markdown and simply edit the connected markdown file in the ./docs/assets/ folder. Then restart dapperdox and you should see the changes on your site.

In order to understand the folder hierachie in dapperboxs assests folder, read: 
http://dapperdox.io/docs/author-concepts

Dapperbox needs a swagger.json file in the specification folder. So make sure to use the newest openapi.yaml for RLL and convert it to json if necessery. (e.g https://www.json2yaml.com/)
   

#### Misc
Generate Static Pages: 
https://stackoverflow.com/questions/26605217/generate-static-docs-with-swagger?utm_medium=organic&utm_source=google_rich_qa&utm_campaign=google_rich_qa

Convert to PDF Issue: 
https://github.com/swagger-api/swagger-codegen/issues/4769
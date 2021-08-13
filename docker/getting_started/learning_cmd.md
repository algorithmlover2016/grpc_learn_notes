Docker Tutorial: https://github.com/docker/getting-started
After create the docker-tutorial container, the turorial will be: Getting Started

Create volume:
	Named volume: From <http://localhost:81/tutorial/persisting-our-data/>
		docker volume create todo-db
		docker run -dp 3000:3000 -v todo-db:/etc/todos getting-started
		docker volume inspect todo-db
	Bind Mounts: From <http://localhost:81/tutorial/using-bind-mounts/> 
		Unix:
			docker run -dp 3000:3000 \
			    -w /app -v "$(pwd):/app" \
			    node:12-alpine \
			    sh -c "yarn install && yarn run dev"
		Windows:
			docker run -dp 3000:3000 `
			    -w /app -v "$(pwd):/app" `
			    node:12-alpine `
			    sh -c "yarn install && yarn run dev"
Create network and connect different containers by network: From <http://localhost:81/tutorial/multi-container-apps/> 
	Create netWork:
		docker network create todo-app
	Create and start mysql:
		Unix:
			docker run -d \
			    --network todo-app --network-alias mysql \
			    -v todo-mysql-data:/var/lib/mysql \
			    -e MYSQL_ROOT_PASSWORD=secret \
			    -e MYSQL_DATABASE=todos \
			    mysql:5.7
		Windows:
			docker run -d `
			    --network todo-app --network-alias mysql `
			    -v todo-mysql-data:/var/lib/mysql `
			    -e MYSQL_ROOT_PASSWORD=secret `
			    -e MYSQL_DATABASE=todos `
			    mysql:5.7
		Test mysql:
			docker exec -it <mysql-container-id> mysql -p
	Create another app container to find mysql:
		docker run -it --network todo-app nicolaka/netshoot
	Create another app container to use mysql
		docker run -dp 3000:3000 \
		  -w /app -v "$(pwd):/app" \
		  --network todo-app \
		  -e MYSQL_HOST=mysql \
		  -e MYSQL_USER=root \
		  -e MYSQL_PASSWORD=secret \
		  -e MYSQL_DB=todos \
		  node:12-alpine \
		  sh -c "yarn install && yarn run dev"
	Notice:
		If we want to change mysql location without restart todo_app, then we need create a new mysql container and create the table of todo_items. 
		 CREATE TABLE `todo_items` ( `id` varchar(36) DEFAULT NULL, `name` varchar(255) DEFAULT NULL, `completed` tinyint(1) DEFAULT NULL) ENGINE=InnoDB DEFAULT CHARSET=latin1;
